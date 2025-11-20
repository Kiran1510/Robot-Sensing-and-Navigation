#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from datetime import datetime, timezone

import serial
import utm
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from gps_msgs.msg import Customgps 


def dm_to_decimal(dm_value: str, hemisphere: str) -> float:
    """
    Convert NMEA degrees+minutes to decimal degrees.

    NMEA format:
      - Latitude:  ddmm.mmmm with hemisphere 'N' or 'S'
      - Longitude: dddmm.mmmm with hemisphere 'E' or 'W'

    """
    if not dm_value:
        return float("nan")

    deg_digits = 2 if hemisphere in ("N", "S") else 3

    try:
        degrees = int(dm_value[:deg_digits])
        minutes = float(dm_value[deg_digits:])
    except (ValueError, IndexError):
        return float("nan")

    decimal = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        decimal = -decimal
    return decimal


def parse_gpgga(line: str):
    """
    Parse a $GPGGA line.

    Returns:
      (utc_time_str, latitude_deg, longitude_deg, altitude_m, hdop)  or  None
    """
    if not line.startswith("$GPGGA"):
        return None

    parts = line.split(",")
    # the GPGGA string expects minimal requirements of:
    #  0:$GPGGA 1:time 2:lat 3:N/S 4:lon 5:E/W 6:fix 7:sats 8:HDOP 9:alt 10:M ...
    if len(parts) < 11:
        return None

    try:
        utc_time = parts[1]  # "hhmmss" or "hhmmss.sss"
        lat = dm_to_decimal(parts[2], parts[3])
        lon = dm_to_decimal(parts[4], parts[5])
        hdop = float(parts[8]) if parts[8] else float("nan")
        alt = float(parts[9]) if parts[9] else float("nan")
        return utc_time, lat, lon, alt, hdop
    except (ValueError, IndexError):
        return None


def gps_time_of_day_to_epoch_time_msg(utc_hms: str) -> Time:
    if not utc_hms:
        now = datetime.now(timezone.utc)
        t = Time()
        t.sec = int(now.timestamp())
        t.nanosec = int((now.timestamp() - t.sec) * 1e9)
        return t

    try:
        h = int(utc_hms[0:2] or 0)
        m = int(utc_hms[2:4] or 0)
        s_float = float(utc_hms[4:] or 0.0)
    except (ValueError, IndexError):
        now = datetime.now(timezone.utc)
        t = Time()
        t.sec = int(now.timestamp())
        t.nanosec = int((now.timestamp() - t.sec) * 1e9)
        return t

    s_int = int(s_float)
    nsec = int((s_float - s_int) * 1e9)

    today = datetime.now(timezone.utc).date()
    dt = datetime(today.year, today.month, today.day, h, m, s_int, tzinfo=timezone.utc)
    sec = int(dt.timestamp())

    t = Time()
    t.sec = sec
    t.nanosec = nsec
    return t


class GPSPublisher(Node):
    def __init__(self):
        super().__init__("gps_publisher")

        # Declare parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 4800)
        self.declare_parameter("read_period_sec", 0.2)
        
        chosen_port = self._get_port_from_args() or \
            self.get_parameter("port").get_parameter_value().string_value
        baud = int(self.get_parameter("baudrate").value)

        if not chosen_port:
            self.get_logger().error("Serial port not defined.")
            rclpy.shutdown()
            return

        # Try opening serial
        try:
            self.ser = serial.Serial(chosen_port, baud, timeout=3.0)
            self.get_logger().info(f"Connected to {chosen_port} @ {baud} baud.")
        except serial.SerialException as err:
            self.get_logger().error(f"Failed to open serial port {chosen_port}: {err}")
            self.ser = None
            rclpy.shutdown()
            return

        # Publisher & timer
        self.pub = self.create_publisher(Customgps, "/gps", 10)
        period = float(self.get_parameter("read_period_sec").value)
        self.timer = self.create_timer(period, self._poll_and_publish)
        self.last_rx_time = self.get_clock().now()

    def _get_port_from_args(self):
        for arg in sys.argv:
            if arg.startswith("port:"):
                return arg.split(":", 1)[1]
        return None

    def _poll_and_publish(self):
        if self.ser is None:
            self.get_logger().warn("Serial not available.")
            return

        try:
            raw = self.ser.readline().decode("ascii", errors="ignore").strip()
        except serial.SerialException as err:
            self.get_logger().error(f"Serial read error: {err}")
            return

        if not raw:
            return

        if raw.startswith("$GPGGA"):
            parsed = parse_gpgga(raw)
            if parsed is None:
                return

            utc_time, lat, lon, alt, hdop = parsed

            # Convert to UTM
            try:
                utm_e, utm_n, zone, letter = utm.from_latlon(lat, lon)
            except Exception as e:
                self.get_logger().warn(f"UTM conversion failed for lat={lat}, lon={lon}: {e}")
                return

            # Build Customgps message
            msg = Customgps()
            msg.header = Header()
            msg.header.frame_id = "GPS1_Frame"
            msg.header.stamp = gps_time_of_day_to_epoch_time_msg(utc_time)

            msg.latitude = float(lat)
            msg.longitude = float(lon)
            msg.altitude = float(alt)
            msg.utm_easting = float(utm_e)
            msg.utm_northing = float(utm_n)
            msg.zone = int(zone)         
            msg.letter = str(letter)       
            msg.hdop = float(hdop)
            msg.gpgga_read = raw            

            self.pub.publish(msg)
            self.last_rx_time = self.get_clock().now()

            self.get_logger().info(
                f"GPGGA → lat:{lat:.6f} lon:{lon:.6f} alt:{alt:.2f} "
                f"UTM(E,N,Z,B)=({utm_e:.3f}, {utm_n:.3f}, {zone}, {letter}) hdop:{hdop:.2f}"
            )

        #warn if no data in > 5 s
        if (self.get_clock().now() - self.last_rx_time).nanoseconds > 5e9:
            self.get_logger().warn("No GPGGA received in the last 5 seconds.")

    def destroy_node(self):
        try:
            if getattr(self, "ser", None) and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    try:
        if getattr(node, "ser", None) is not None:
            rclpy.spin(node)
        else:
            node.get_logger().error("GPS node not running (serial unavailable).")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

