#!/usr/bin/env python3
# -*- coding: utf-8 -*-


#vectornav VN-100 ROS2 driver (publishes vn_driver/msg/Vectornav on /imu)
#reads $VNYMR from a serial port (115200 8N1)
#validates checksum, parses VN order, remaps to ROS ENU
#converts Euler (deg) -> quaternion; gyro deg/s -> rad/s; mag Gauss -> Tesla
#publishes on line arrival; optionally configures & confirms 40 Hz on startup


import math
import threading
import serial
import rclpy
from rclpy.node import Node
from vn_driver.msg import Vectornav  # custom message in this package

DEG2RAD = math.pi / 180.0
GAUSS_TO_TESLA = 1e-4

def convert_to_quaternion(yaw_deg: float, pitch_deg: float, roll_deg: float):
    cy, sy = math.cos(yaw_deg * DEG2RAD / 2.0), math.sin(yaw_deg * DEG2RAD / 2.0)
    cp, sp = math.cos(pitch_deg * DEG2RAD / 2.0), math.sin(pitch_deg * DEG2RAD / 2.0)
    cr, sr = math.cos(roll_deg * DEG2RAD / 2.0), math.sin(roll_deg * DEG2RAD / 2.0)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w

def valid_vnymr_checksum(line: str) -> bool:
    if not line or line[0] != '$' or '*' not in line:
        return False
    payload = line[1: line.rfind('*')]
    cs = line[line.rfind('*')+1:][:2]
    try:
        want = int(cs, 16)
    except ValueError:
        return False
    calc = 0
    for ch in payload:
        calc ^= ord(ch)
    return calc == want

def ned_vec_to_enu(x_n, y_n, z_n):
    return y_n, x_n, -z_n

def ned_ypr_to_enu_rpy(yaw_n, pitch_n, roll_n):
    roll_e  =  pitch_n
    pitch_e =  roll_n
    yaw_e   = -yaw_n
    return yaw_e, pitch_e, roll_e

def _xor_checksum(payload: str) -> str:
    cs = 0
    for ch in payload:
        cs ^= ord(ch)
    return f"{cs:02X}"

def configure_imu_rate(ser: serial.Serial, rate_hz: int) -> None:
    payload = f"VNWRG,07,{rate_hz}"
    cmd = f"${payload}*{_xor_checksum(payload)}\r\n"
    ser.write(cmd.encode('utf-8'))
    ser.flush()

def query_imu_rate(ser: serial.Serial, logger) -> int | None:
    payload = "VNRRG,07"
    cmd = f"${payload}*{_xor_checksum(payload)}\r\n"
    old_to = ser.timeout
    try:
        ser.timeout = 0.2
        ser.reset_input_buffer()
        ser.write(cmd.encode('utf-8'))
        ser.flush()
        for _ in range(25):
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line or not line.startswith('$'):
                continue
            # something like: $VNRRG,07,<rate>*CS
            if line.startswith("$VNRRG,07,"):
                try:
                    head = line[: line.rfind('*')]
                    parts = head.split(',')
                    if len(parts) >= 3:
                        rate = float(parts[2])
                        logger.info(f"[VN] Reported async rate (reg 07): {rate} Hz")
                        return int(round(rate))
                except Exception:
                    logger.warn(f"[VN] Could not parse rate from: {line}")
                    return None
        logger.warn("[VN] No readable response to VNRRG,07")
        return None
    finally:
        ser.timeout = old_to

class IMUDriver(Node):
    def __init__(self):
        super().__init__('vn100_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('set_rate_on_startup', True)
        self.declare_parameter('target_rate_hz', 40)

        serial_port = self.get_parameter('port').get_parameter_value().string_value
        serial_baud = int(self.get_parameter('baudrate').get_parameter_value().integer_value or 115200)
        set_rate    = bool(self.get_parameter('set_rate_on_startup').get_parameter_value().bool_value)
        target_rate = int(self.get_parameter('target_rate_hz').get_parameter_value().integer_value or 40)

        try:
            self.ser = serial.Serial(serial_port, serial_baud, timeout=0.05)
        except Exception as e:
            self.get_logger().fatal(f"Failed to open port {serial_port} @ {serial_baud}: {e}")
            raise

        if set_rate:
            configure_imu_rate(self.ser, target_rate)
            _ = query_imu_rate(self.ser, self.get_logger())

        self.pub = self.create_publisher(Vectornav, 'imu', 10)
        self.msg = Vectornav()
        self.msg.header.frame_id = "imu1_frame"
        self.msg.imu.header.frame_id = "imu1_frame"
        self.msg.mag_field.header.frame_id = "imu1_frame"

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f"Publishing IMU readings from {serial_port}")

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
            except Exception as e:
                self.get_logger().warn(f"serial read error: {e}")
                continue
            if not raw:
                continue
            if not raw.startswith('$VNYMR') or '*' not in raw:
                continue
            if not valid_vnymr_checksum(raw):
                continue
            try:
                core = raw[: raw.rfind('*')]
                fields = core.split(',')
                if len(fields) != 13:
                    continue

                _, yaw_n, pitch_n, roll_n, magX_n, magY_n, magZ_n, accX_n, accY_n, accZ_n, gyrX_n, gyrY_n, gyrZ_n = fields
                yaw_n   = float(yaw_n);   pitch_n = float(pitch_n); roll_n  = float(roll_n)
                magX_n, magY_n, magZ_n = float(magX_n), float(magY_n), float(magZ_n)
                accX_n, accY_n, accZ_n = float(accX_n), float(accY_n), float(accZ_n)
                gyrX_n, gyrY_n, gyrZ_n = float(gyrX_n), float(gyrY_n), float(gyrZ_n)

                accX_e, accY_e, accZ_e = ned_vec_to_enu(accX_n, accY_n, accZ_n)
                gyrX_e, gyrY_e, gyrZ_e = ned_vec_to_enu(gyrX_n, gyrY_n, gyrZ_n)
                magX_e, magY_e, magZ_e = ned_vec_to_enu(magX_n, magY_n, magZ_n)

                gyrX_e *= DEG2RAD; gyrY_e *= DEG2RAD; gyrZ_e *= DEG2RAD
                magX_e *= GAUSS_TO_TESLA; magY_e *= GAUSS_TO_TESLA; magZ_e *= GAUSS_TO_TESLA

                yaw_e, pitch_e, roll_e = ned_ypr_to_enu_rpy(yaw_n, pitch_n, roll_n)
                qx, qy, qz, qw = convert_to_quaternion(yaw_e, pitch_e, roll_e)

                now = self.get_clock().now().to_msg()
                self.msg.raw = raw
                self.msg.header.stamp = now
                self.msg.imu.header.stamp = now
                self.msg.mag_field.header.stamp = now

                self.msg.imu.orientation.x = qx
                self.msg.imu.orientation.y = qy
                self.msg.imu.orientation.z = qz
                self.msg.imu.orientation.w = qw

                self.msg.imu.angular_velocity.x = gyrX_e
                self.msg.imu.angular_velocity.y = gyrY_e
                self.msg.imu.angular_velocity.z = gyrZ_e

                self.msg.imu.linear_acceleration.x = accX_e
                self.msg.imu.linear_acceleration.y = accY_e
                self.msg.imu.linear_acceleration.z = accZ_e

                self.msg.mag_field.magnetic_field.x = magX_e
                self.msg.mag_field.magnetic_field.y = magY_e
                self.msg.mag_field.magnetic_field.z = magZ_e

                self.pub.publish(self.msg)

            except Exception as e:
                self.get_logger().warn(f"parse/publish error: {e}")
                continue

    def destroy_node(self):
        try:
            self._stop.set()
            if hasattr(self, "_thread"):
                self._thread.join(timeout=0.5)
        finally:
            try:
                if hasattr(self, "ser") and self.ser:
                    self.ser.close()
            except Exception:
                pass
            super().destroy_node()

def main():
    rclpy.init()
    node = IMUDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
