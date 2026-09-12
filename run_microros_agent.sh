#!/usr/bin/env bash
# Runs the micro-ROS agent via the official Docker image, bridging the
# ESP32's serial link into this host's ROS2 graph (DDS). No local build
# needed - building the Agent from source (eProsima's SuperBuild, which
# compiles Fast-DDS/Fast-CDR from source) turned out flaky on this host
# across three different failure modes (a system spdlog/fmt conflict, an
# OOM kill with no swap, and a broken checkout off a moving branch), so we
# use the prebuilt image instead.
#
# Usage: ./run_microros_agent.sh [serial-device]
#   serial-device defaults to /dev/ttyTHS1 (or $ESP32_SERIAL_PORT) - the
#   Jetson's header UART1 (physical pins 8 TXD / 10 RXD), wired directly
#   to the ESP32's dedicated micro-ROS UART (see ros2_control_app.inc).
#   Pass /dev/esp32maker explicitly for USB-transport apps instead
#   (chatter_microros_app.inc still uses the USB/CH340 line).
set -euo pipefail

PORT="${1:-${ESP32_SERIAL_PORT:-/dev/ttyTHS1}}"
if [ ! -e "$PORT" ]; then
  echo "$PORT not found." >&2
  echo "If using the default /dev/ttyTHS1, check the Jetson<->ESP32 UART" >&2
  echo "wiring; if using /dev/esp32maker, run" >&2
  echo "../JetsonCortex/install_ch341_driver first - or pass the right" >&2
  echo "serial device as an argument." >&2
  exit 1
fi

# --privileged: relaxes the container's device cgroup so it's *allowed* to
# open host devices. -v "$PORT":"$PORT": actually makes it visible -
# containers get their own private /dev by default, which doesn't include
# host udev symlinks like /dev/esp32maker; bind-mounting the path (rather
# than using --device, which doesn't resolve symlinks) follows it through
# to the real device. --privileged alone isn't enough for either.
# --ipc=host: shares the host's IPC namespace so Fast-DDS's shared-memory
# transport works between the Agent and ROS2 nodes on this host.
# --net=host: so the Agent's DDS traffic reaches ROS2 nodes on this host
# (rclpy publishers/subscribers use normal DDS discovery, not a container
# network namespace).
exec docker run --rm --privileged -v "$PORT":"$PORT" --ipc=host --net=host \
  microros/micro-ros-agent:jazzy \
  serial --dev "$PORT" -b 115200 -v4
