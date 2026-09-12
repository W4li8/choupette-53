#!/usr/bin/env python3
"""Combines both lidar point clouds into one stop/continue command.

Subscribes to /lidar/i2c_0x29/points (VL53L0X single-point sensor) and
/lidar/i2c_0x33/points (VL53L7CX 8x8 matrix sensor - see
lidar_point_node.cpp/lidar_matrix_node.cpp), applies the same distance
thresholds the ESP32 firmware used to apply locally
(src/apps/lidar_point_app.inc, src/apps/lidar_matrix_app.inc), and
publishes std_msgs/Bool on /safety_stop (true = stop) for
ros2_control_app.inc to subscribe to over micro-ROS.

Fails stopped, matching the ESP32 firmware's own instinct: either topic
going stale (sensor lost, node crashed, etc.) is treated the same as an
obstacle - "no data" is never "clear to go".

Run via `pixi run -e ros2 safety_stop`.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from std_msgs.msg import Bool

# Matches OBSTACLE_THRESHOLD_MM in both ESP32 lidar apps (100mm / 10cm).
POINT_OBSTACLE_THRESHOLD_M = 0.1
MATRIX_OBSTACLE_THRESHOLD_M = 0.1

# A sensor topic going quiet this long (crashed node, lost sensor, i2c
# bus fault) is treated as an obstacle, not as "nothing to report".
STALE_TIMEOUT_S = 0.5

CHECK_PERIOD_S = 0.1
STOP_TOPIC = "/safety_stop"


def _closest_distance_m(msg: PointCloud2):
    points = read_points_numpy(msg, field_names=("x", "y", "z"))
    if points.shape[0] == 0:
        return None
    return np.linalg.norm(points, axis=1)


class LidarSafetyStopNode(Node):
    def __init__(self):
        super().__init__("lidar_safety_stop_node")

        self._point_close = None  # None = no reading yet
        self._matrix_close = None
        self._last_point_stamp = None
        self._last_matrix_stamp = None

        self.create_subscription(PointCloud2, "/lidar/i2c_0x29/points", self._point_cb, 10)
        self.create_subscription(PointCloud2, "/lidar/i2c_0x33/points", self._matrix_cb, 10)
        self._pub = self.create_publisher(Bool, STOP_TOPIC, 10)
        self.create_timer(CHECK_PERIOD_S, self._evaluate)

    def _point_cb(self, msg: PointCloud2):
        distances = _closest_distance_m(msg)
        # No points at all reads the same as "too close" - a driver that
        # stops publishing valid points has, by definition, stopped
        # confirming it's safe to go.
        self._point_close = distances is None or bool(np.any(distances < POINT_OBSTACLE_THRESHOLD_M))
        self._last_point_stamp = self.get_clock().now()

    def _matrix_cb(self, msg: PointCloud2):
        distances = _closest_distance_m(msg)
        # Matches the ESP32 firmware's ALL-zones-close semantics exactly
        # (kept on request, even though the firmware's own comment notes
        # this condition may rarely trip given how unevenly real scenes
        # read across 64 zones).
        self._matrix_close = distances is None or bool(np.all(distances < MATRIX_OBSTACLE_THRESHOLD_M))
        self._last_matrix_stamp = self.get_clock().now()

    def _is_stale(self, last_stamp) -> bool:
        if last_stamp is None:
            return True
        age_s = (self.get_clock().now() - last_stamp).nanoseconds / 1e9
        return age_s > STALE_TIMEOUT_S

    def _evaluate(self):
        stop = (
            self._is_stale(self._last_point_stamp)
            or self._is_stale(self._last_matrix_stamp)
            or bool(self._point_close)
            or bool(self._matrix_close)
        )
        msg = Bool()
        msg.data = stop
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = LidarSafetyStopNode()
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
