// Real ROS2 control: subscribes to /cmd_vel (geometry_msgs/Twist,
// differential drive - M1+M2 left, M3+M4 right, see motor_control.inc)
// and /cmd_rgb (std_msgs/ColorRGBA) over the micro-ROS agent link (see
// ../../run_microros_agent.sh).
//
// Same USB serial caveat as chatter_microros_app.inc: the line carries
// XRCE-DDS traffic, not debug text, so there's nothing to Serial.println
// here either - use `ros2 topic pub /cmd_vel ...` / `ros2 topic pub
// /cmd_rgb ...` (or a real teleop node) and watch the robot itself.
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/color_rgba.h>

#include "motor_control.inc"

namespace {

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

rcl_subscription_t cmd_vel_sub;
rcl_subscription_t cmd_rgb_sub;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__ColorRGBA cmd_rgb_msg;

// Safety watchdog: a publisher that dies (crashed teleop node, dropped
// connection) shouldn't leave the robot driving at its last command
// forever. Mirrors the lidar apps' instinct to fail stopped, not moving.
const unsigned long CMD_VEL_TIMEOUT_MS = 500;
volatile unsigned long lastCmdVelMillis = 0;

void errorLoop() {
  while (true) {
    delay(100);
  }
}

#define RCCHECK(fn)                  \
  do {                               \
    if ((fn) != RCL_RET_OK) {        \
      errorLoop();                   \
    }                                \
  } while (0)

void cmdVelCallback(const void *msgin) {
  const auto *msg = static_cast<const geometry_msgs__msg__Twist *>(msgin);
  driveDifferential(msg->linear.x, msg->angular.z);
  lastCmdVelMillis = millis();
}

void cmdRgbCallback(const void *msgin) {
  const auto *msg = static_cast<const std_msgs__msg__ColorRGBA *>(msgin);
  setRgbColor(msg->r, msg->g, msg->b, msg->a);
}

}  // namespace

void setup() {
  set_microros_serial_transports(Serial);
  Serial.begin(115200);
  delay(2000);  // let the micro-ROS agent's serial port settle

  initMotorsAndLeds();

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_control_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_default(
      &cmd_rgb_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA), "cmd_rgb"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
                                          &cmdVelCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_rgb_sub, &cmd_rgb_msg,
                                          &cmdRgbCallback, ON_NEW_DATA));

  lastCmdVelMillis = millis();
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  if (millis() - lastCmdVelMillis > CMD_VEL_TIMEOUT_MS) {
    driveDifferential(0, 0);
  }
}
