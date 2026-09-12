// micro-ROS version of the chatter smoke test: a real rclc node, not a
// raw text echo (see chatter_app.inc for that simpler physical-layer-only
// version). Subscribes to /chatter (std_msgs/String, published by
// ../../ros2_chatter_publisher.py) and republishes an echo on
// /chatter_echo - watch it with `ros2 topic echo /chatter_echo`.
//
// The USB serial line now carries the binary micro-ROS/XRCE-DDS protocol
// itself (see ../../run_microros_agent.sh), so there's no room left on it
// for human-readable Serial.println() debugging like the other apps use -
// that's why the round trip is verified via a ROS2 topic instead of the
// serial monitor.
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

namespace {

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

rcl_subscription_t subscriber;
rcl_publisher_t echo_publisher;

std_msgs__msg__String recv_msg;
std_msgs__msg__String echo_msg;
char recv_buf[128];
char echo_buf[160];

void errorLoop() {
  // Nothing else we can report over - the serial line is busy carrying
  // XRCE-DDS traffic, not debug text.
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

void subscriptionCallback(const void *msgin) {
  const auto *msg = static_cast<const std_msgs__msg__String *>(msgin);
  snprintf(echo_buf, sizeof(echo_buf), "esp32 got: %s", msg->data.data);
  echo_msg.data.size = strlen(echo_buf);
  rcl_publish(&echo_publisher, &echo_msg, NULL);
}

}  // namespace

void setup() {
  set_microros_serial_transports(Serial);
  Serial.begin(115200);
  delay(2000);  // let the micro-ROS agent's serial port settle

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_chatter_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "chatter"));
  RCCHECK(rclc_publisher_init_default(
      &echo_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "chatter_echo"));

  recv_msg.data.data = recv_buf;
  recv_msg.data.capacity = sizeof(recv_buf);
  recv_msg.data.size = 0;

  echo_msg.data.data = echo_buf;
  echo_msg.data.capacity = sizeof(echo_buf);
  echo_msg.data.size = 0;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg,
                                          &subscriptionCallback, ON_NEW_DATA));
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
