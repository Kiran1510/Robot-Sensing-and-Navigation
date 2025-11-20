#!/usr/bin/env python3
import sys
import serial
import utm
from datetime import datetime, timezone
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from gps_driver.msg import Customrtk


#convert ddmm.mmmm to decimal degrees
def dm_to_decimal(dm_value: str, hemi: str) -> float:
    if not dm_value:
        return float("nan")
    d = 2 if hemi in ("N", "S") else 3
    degrees = int(dm_value[:d])
    minutes = float(dm_value[d:])
    dec = degrees + minutes / 60.0
    return -dec if hemi in ("S", "W") else dec


#parse gngga line
def parse_gngga(line: str):
    if not line.startswith("$GNGGA"):
        return None
    p = line.split(",")
    if len(p) < 10:
        return None
    try:
        utc = p[1]
        lat = dm_to_decimal(p[2], p[3])
        lon = dm_to_decimal(p[4], p[5])
        fix = int(p[6] or 0)
        hdop = float(p[8]) if p[8] else float("nan")
        alt = float(p[9]) if p[9] else float("nan")
        return utc, lat, lon, alt, hdop, fix
    except Exception:
        return None


#convert gps utc time to epoch
def utc_to_epoch_msg(utc_hms: str) -> Time:
    t = Time()
    try:
        h, m = int(utc_hms[0:2]), int(utc_hms[2:4])
        s = float(utc_hms[4:]) if utc_hms[4:] else 0.0
        sec = int(s)
        nsec = int((s - sec) * 1e9)
        today = datetime.now(timezone.utc).date()
        epoch = datetime(today.year, today.month, today.day, h, m, sec,
                         tzinfo=timezone.utc).timestamp()
    except Exception:
        epoch, nsec = datetime.now(timezone.utc).timestamp(), 0
    t.sec = int(epoch)
    t.nanosec = nsec
    return t


class RTKPublisher(Node):
    def __init__(self):
        super().__init__("rtk_publisher")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("read_period_sec", 1.0 / 15)

        port = self._cli_port() or self.get_parameter("port").get_parameter_value().string_value
        baud = int(self.get_parameter("baudrate").value)
        period = float(self.get_parameter("read_period_sec").value)

        try:
            self.ser = serial.Serial(port, baud, timeout=2.0)
            self.get_logger().info(f"RTK connected: {port} @ {baud} baud")
        except serial.SerialException as e:
            raise RuntimeError(f"Port error: {e}")

        self.pub = self.create_publisher(Customrtk, "/gps", 10)
        self.timer = self.create_timer(period, self.read_once)

    def _cli_port(self):
        for arg in sys.argv:
            if arg.startswith("port:"):
                return arg.split(":", 1)[1]
        return None

    def read_once(self):
        try:
            line = self.ser.readline().decode("ascii", errors="ignore").strip()
            if not line or not line.startswith("$GNGGA"):
                return

            parsed = parse_gngga(line)
            if not parsed:
                return
            utc, lat, lon, alt, hdop, fix = parsed
            utm_e, utm_n, zone, letter = utm.from_latlon(lat, lon)

            msg = Customrtk()
            msg.header = Header(frame_id="GPS1_Frame", stamp=utc_to_epoch_msg(utc))
            msg.latitude, msg.longitude, msg.altitude = lat, lon, alt
            msg.utm_easting, msg.utm_northing = utm_e, utm_n
            msg.zone, msg.letter = zone, letter
            msg.fix_quality, msg.hdop, msg.gngga_read = fix, hdop, line
            self.pub.publish(msg)

            #print parsed data
            self.get_logger().info(
                f"GNGGA -> lat:{lat:.7f} lon:{lon:.7f} alt:{alt:.2f} "
                f"UTM(E,N,Z,B)=({utm_e:.3f}, {utm_n:.3f}, {zone}, {letter}) "
                f"fix:{fix} hdop:{hdop:.2f}"
            )

        except Exception as e:
            self.get_logger().warn(f"Error reading RTK data: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RTKPublisher()
    except Exception as e:
        print(f"[rtk_publisher] initialization failed: {e}")
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if getattr(node, "ser", None):
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
