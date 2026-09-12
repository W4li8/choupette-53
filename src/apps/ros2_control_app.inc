// Real ROS2 control: subscribes to /cmd_vel (geometry_msgs/Twist,
// differential drive - M1+M2 left, M3+M4 right, see motor_control.inc)
// and /cmd_rgb (std_msgs/ColorRGBA) over the micro-ROS agent link (see
// ../../run_microros_agent.sh).
//
// The XRCE-DDS link runs over a dedicated hardware UART (Serial1 on
// MICROROS_UART_RX_PIN/TX_PIN) wired directly to the Jetson's header UART1
// (physical pins 8 TXD / 10 RXD -> /dev/ttyTHS1), not the USB/CH340 line -
// that keeps flashing and micro-ROS traffic on separate wires, and frees
// up USB Serial for actual Serial.println debugging if needed, unlike the
// USB-transport apps (chatter_microros_app.inc) where the line is busy
// carrying the binary protocol itself.
#include <Arduino.h>
#include <micro_ros_platformio.h>

// ESP32 RX <- Jetson UART1 TXD (header pin 8); ESP32 TX -> Jetson UART1
// RXD (header pin 10).
#define MICROROS_UART_RX_PIN 19
#define MICROROS_UART_TX_PIN 18

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

// USB Serial is free (the XRCE-DDS link is on Serial1 - see above), so
// debug logging goes out over it: watch it with `pio device monitor` or
// any serial terminal at 115200 on /dev/esp32maker.
void errorLoop(const char *step) {
  Serial.print("ros2_control: FAILED at ");
  Serial.print(step);
  Serial.print(": ");
  Serial.println(rcl_get_error_string().str);
  rcl_reset_error();
  while (true) {
    delay(100);
  }
}

#define RCCHECK(fn)                  \
  do {                               \
    if ((fn) != RCL_RET_OK) {        \
      errorLoop(#fn);                \
    }                                \
  } while (0)

void cmdVelCallback(const void *msgin) {
  const auto *msg = static_cast<const geometry_msgs__msg__Twist *>(msgin);
  driveDifferential(msg->linear.x, msg->angular.z);
  lastCmdVelMillis = millis();
  Serial.printf("ros2_control: cmd_vel linear.x=%.3f angular.z=%.3f\n",
                msg->linear.x, msg->angular.z);
}

void cmdRgbCallback(const void *msgin) {
  const auto *msg = static_cast<const std_msgs__msg__ColorRGBA *>(msgin);
  setRgbColor(msg->r, msg->g, msg->b, msg->a);
  Serial.printf("ros2_control: cmd_rgb r=%.3f g=%.3f b=%.3f a=%.3f\n",
                msg->r, msg->g, msg->b, msg->a);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, MICROROS_UART_RX_PIN, MICROROS_UART_TX_PIN);
  set_microros_serial_transports(Serial1);
  delay(2000);  // let the micro-ROS agent's serial port settle
  Serial.println("ros2_control: booted, Serial1 up on RX=19/TX=18, starting micro-ROS init");

  initMotorsAndLeds();

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  Serial.println("ros2_control: rclc_support_init OK (agent session established)");
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
  Serial.println("ros2_control: node+subscriptions ready, entering loop()");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  if (millis() - lastCmdVelMillis > CMD_VEL_TIMEOUT_MS) {
    driveDifferential(0, 0);
  }
}
