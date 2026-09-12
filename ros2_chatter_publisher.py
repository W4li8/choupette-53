#!/usr/bin/env python3
"""Publishes std_msgs/String "chatter N" on /chatter once a second.

Requires the micro-ROS agent bridging the ESP32's serial link into this
ROS2 graph - see ./run_microros_agent.sh (run in a separate terminal).
The ESP32 (src/apps/chatter_microros_app.inc,
built via the `chatter_microros` platformio env) subscribes to /chatter and
republishes an echo on /chatter_echo - watch it with
`ros2 topic echo /chatter_echo` (pixi run -e ros2 ros2 topic echo /chatter_echo).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ChatterPublisher(Node):
    def __init__(self):
        super().__init__("chatter_publisher")
        self._pub = self.create_publisher(String, "chatter", 10)
        self._count = 0
        self.create_timer(1.0, self._tick)

    def _tick(self):
        msg = String()
        msg.data = f"chatter {self._count}"
        self._pub.publish(msg)
        self.get_logger().info(f"-> {msg.data}")
        self._count += 1


def main():
    rclpy.init()
    node = ChatterPublisher()
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
