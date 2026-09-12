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
#include <rmw_microros/rmw_microros.h>
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

// Connection state machine (standard micro-ROS Arduino reconnection
// pattern): rclc_support_init only ever attempts the agent handshake
// once, so without this a boot (or a later drop, e.g. the agent process
// dying/restarting) before the agent is reachable would latch forever -
// there's no self-healing otherwise. WAITING_AGENT/AGENT_AVAILABLE poll
// rmw_uros_ping_agent (a lightweight liveness check, not the actual
// XRCE-DDS session) before spending time on the full entity setup;
// AGENT_CONNECTED keeps pinging periodically so a lost heartbeat is
// noticed and triggers a clean teardown + retry from scratch.
enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED,
};
AgentState state = WAITING_AGENT;

const int PING_TIMEOUT_MS = 100;
const uint8_t PING_ATTEMPTS = 1;
const unsigned long AGENT_POLL_INTERVAL_MS = 500;
unsigned long lastAgentPollMillis = 0;

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

// Returns false (without halting) on the first failed step, so a
// mid-setup failure (e.g. the agent vanishing between the ping and node
// creation) just sends us back through WAITING_AGENT instead of hanging.
bool createEntities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) goto fail;
  if (rclc_node_init_default(&node, "esp32_control_node", "", &support) != RCL_RET_OK) goto fail;

  if (rclc_subscription_init_default(
          &cmd_vel_sub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) {
    goto fail;
  }
  if (rclc_subscription_init_default(
          &cmd_rgb_sub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA), "cmd_rgb") != RCL_RET_OK) {
    goto fail;
  }

  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) goto fail;
  if (rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
                                      &cmdVelCallback, ON_NEW_DATA) != RCL_RET_OK) {
    goto fail;
  }
  if (rclc_executor_add_subscription(&executor, &cmd_rgb_sub, &cmd_rgb_msg,
                                      &cmdRgbCallback, ON_NEW_DATA) != RCL_RET_OK) {
    goto fail;
  }

  lastCmdVelMillis = millis();
  Serial.println("ros2_control: entities created, agent session established");
  return true;

fail:
  Serial.print("ros2_control: entity creation failed: ");
  Serial.println(rcl_get_error_string().str);
  rcl_reset_error();
  return false;
}

// Safe to call even on a support/node that's only partially constructed
// (createEntities() bailing out midway) - each fini is independently
// no-op-safe on a zero-initialized handle.
void destroyEntities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&cmd_vel_sub, &node);
  rcl_subscription_fini(&cmd_rgb_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, MICROROS_UART_RX_PIN, MICROROS_UART_TX_PIN);
  set_microros_serial_transports(Serial1);
  delay(2000);  // let the micro-ROS agent's serial port settle
  Serial.println("ros2_control: booted, Serial1 up on RX=19/TX=18, waiting for agent");

  initMotorsAndLeds();
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      if (millis() - lastAgentPollMillis > AGENT_POLL_INTERVAL_MS) {
        lastAgentPollMillis = millis();
        state = (RMW_RET_OK == rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS))
                    ? AGENT_AVAILABLE
                    : WAITING_AGENT;
      }
      break;

    case AGENT_AVAILABLE:
      state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroyEntities();
      }
      break;

    case AGENT_CONNECTED:
      if (millis() - lastAgentPollMillis > AGENT_POLL_INTERVAL_MS) {
        lastAgentPollMillis = millis();
        state = (RMW_RET_OK == rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS))
                    ? AGENT_CONNECTED
                    : AGENT_DISCONNECTED;
      }
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;

    case AGENT_DISCONNECTED:
      Serial.println("ros2_control: agent heartbeat lost, tearing down and retrying");
      destroyEntities();
      state = WAITING_AGENT;
      break;
  }

  // Runs regardless of connection state: a stale /cmd_vel (dead teleop,
  // or the agent link itself down) always fails stopped.
  if (millis() - lastCmdVelMillis > CMD_VEL_TIMEOUT_MS) {
    driveDifferential(0, 0);
  }
}
