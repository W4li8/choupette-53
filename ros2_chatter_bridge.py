#!/usr/bin/env python3
"""Minimal ROS2 <-> ESP32 smoke test.

Publishes std_msgs/String "chatter N" on the /chatter topic once a second
(so `ros2 topic echo /chatter` sees it like any other ROS2 topic), and
relays the same text over a serial port to the ESP32 (see
src/apps/chatter_app.inc), which echoes it back on its own Serial output.

Run via `pixi run -e ros2 chatter` (see pixi.toml). Not part of the real
zenoh/micro-ROS pipeline discussed for the robot - just proof the ROS2
environment and the ESP32 can talk over the wire.
"""
import argparse
import os

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_PORT = os.environ.get("ESP32_SERIAL_PORT", "/dev/esp32maker")
BAUD = 115200


class ChatterBridge(Node):
    def __init__(self, ser):
        super().__init__("chatter_bridge")
        self._ser = ser
        self._pub = self.create_publisher(String, "chatter", 10)
        self._count = 0
        self.create_timer(1.0, self._tick)

    def _tick(self):
        echo = self._ser.readline().decode(errors="replace").strip()
        if echo:
            self.get_logger().info(f"<- ESP32: {echo}")

        msg = String()
        msg.data = f"chatter {self._count}"
        self._pub.publish(msg)
        self._ser.write((msg.data + "\n").encode())
        self.get_logger().info(f"-> ROS2 + ESP32: {msg.data}")
        self._count += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=(
            "Serial device for the ESP32 (default: %(default)s, or set "
            "ESP32_SERIAL_PORT). On a fresh machine without the "
            "/dev/esp32maker udev symlink, pass e.g. --port /dev/ttyUSB0."
        ),
    )
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, BAUD, timeout=1)
    except serial.SerialException as e:
        raise SystemExit(
            f"Could not open {args.port}: {e}\n"
            "Pass --port /dev/ttyUSBx if this machine doesn't have the "
            "/dev/esp32maker udev symlink set up."
        )

    with ser:
        rclpy.init()
        node = ChatterBridge(ser)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
