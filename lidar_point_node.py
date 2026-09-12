#!/usr/bin/env python3
"""VL53L0X single-point ToF lidar -> sensor_msgs/PointCloud2.

Jetson-side port of src/apps/lidar_point_app.inc's sensor handling (the
ESP32 firmware stays in place as a fallback - see that file's comments
for the original VL53L0X quirks this preserves). Publishes one point per
reading (this sensor has exactly one beam) in the sensor's own frame,
with the beam along +X (REP-103: x forward) - the sensor's mounting
orientation on the robot is a separate concern, handled by a static
transform (see pixi.toml's lidar_point_tf task) rather than baked in
here.

Uses the Adafruit CircuitPython VL53L0X driver (a well-tested port of
ST's init/calibration sequence) via a minimal smbus2-backed I2C shim
instead of Blinka's board auto-detection, since that's already been
found unreliable on this Jetson (the header's I2C1, physical pins 3/5,
is /dev/i2c-7 here, not /dev/i2c-1 - see project memory).

Run via `pixi run -e ros2 lidar_point`.
"""
import argparse

import adafruit_vl53l0x
import rclpy
import smbus2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

DEFAULT_I2C_BUS = 7
DEFAULT_ADDRESS = 0x29
SENSOR_RETRY_INTERVAL_S = 0.5
IO_TIMEOUT_S = 0.5  # matches the ESP32 firmware's pointSensors[i].setTimeout(500)
READ_PERIOD_S = 1.0 / 30.0  # ~30Hz, matches the ESP32 firmware's measured rate


class SMBus2I2C:
    """Minimal busio.I2C-compatible shim (try_lock/unlock/writeto/
    readfrom_into/writeto_then_readfrom) backed directly by smbus2, so
    adafruit_bus_device.I2CDevice works against a specific /dev/i2c-N
    without going through Blinka's board detection."""

    def __init__(self, bus_num):
        self._bus = smbus2.SMBus(bus_num)

    def try_lock(self):
        return True

    def unlock(self):
        pass

    def writeto(self, address, buffer, *, start=0, end=None, stop=True):
        if end is None:
            end = len(buffer)
        self._bus.i2c_rdwr(smbus2.i2c_msg.write(address, bytes(buffer[start:end])))

    def readfrom_into(self, address, buffer, *, start=0, end=None, stop=True):
        if end is None:
            end = len(buffer)
        msg = smbus2.i2c_msg.read(address, end - start)
        self._bus.i2c_rdwr(msg)
        buffer[start:end] = bytes(msg)

    def writeto_then_readfrom(
        self, address, buffer_out, buffer_in, *,
        out_start=0, out_end=None, in_start=0, in_end=None, stop=False,
    ):
        if out_end is None:
            out_end = len(buffer_out)
        if in_end is None:
            in_end = len(buffer_in)
        wmsg = smbus2.i2c_msg.write(address, bytes(buffer_out[out_start:out_end]))
        rmsg = smbus2.i2c_msg.read(address, in_end - in_start)
        self._bus.i2c_rdwr(wmsg, rmsg)
        buffer_in[in_start:in_end] = bytes(rmsg)


class LidarPointNode(Node):
    def __init__(self, i2c_bus, address):
        super().__init__("lidar_point_node")
        self._i2c = SMBus2I2C(i2c_bus)
        self._address = address
        self._frame_id = f"lidar_i2c_0x{address:02x}"
        self._topic = f"/lidar/i2c_0x{address:02x}/points"
        self._pub = self.create_publisher(PointCloud2, self._topic, 10)
        self._sensor = None
        self.create_timer(READ_PERIOD_S, self._tick)
        self._try_init()

    def _try_init(self):
        try:
            self._sensor = adafruit_vl53l0x.VL53L0X(
                self._i2c, address=self._address, io_timeout_s=IO_TIMEOUT_S
            )
            self._sensor.start_continuous()
            self.get_logger().info(f"sensor ready at 0x{self._address:02x}")
        except (RuntimeError, OSError) as e:
            self._sensor = None
            self.get_logger().warn(f"init failed: {e}, retrying...")

    def _tick(self):
        if self._sensor is None:
            self._try_init()
            return

        try:
            distance_mm = self._sensor.range
        except (RuntimeError, OSError) as e:
            # Same instinct as the ESP32 firmware: a timeout/bus error means
            # we've lost the sensor, so re-run init from scratch rather than
            # trust it's still in a good state.
            self.get_logger().warn(f"read failed: {e}, reinitializing...")
            self._sensor = None
            return

        # VL53L0X hardware quirk (see lidar_point_app.inc): ~8190mm means
        # "no target in range", not an error - published as a normal point,
        # same as any other in-range reading. Downstream (safety-stop node)
        # decides what counts as "too close", not this driver node.
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        point = (distance_mm / 1000.0, 0.0, 0.0)
        self._pub.publish(create_cloud_xyz32(header, [point]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--i2c-bus", type=int, default=DEFAULT_I2C_BUS)
    parser.add_argument("--address", type=lambda s: int(s, 0), default=DEFAULT_ADDRESS)
    args = parser.parse_args()

    rclpy.init()
    node = LidarPointNode(args.i2c_bus, args.address)
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
