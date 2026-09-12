#!/usr/bin/env bash
# Runs zenoh-bridge-ros2dds via the official Docker image: joins this
# host's DDS domain as an ordinary participant (discovering whatever
# topics already exist there - the micro-ROS agent's ESP32 topics,
# rclpy nodes, anything DDS-RTPS, regardless of which RMW created it) and
# mirrors them into zenoh key expressions, so a browser (zenoh-ts over
# WebSocket) can subscribe/publish to the same topics without touching
# ROS2 or DDS directly. See run_microros_agent.sh for the other half of
# the chain (ESP32 <-XRCE-DDS/serial-> agent <-DDS-RTPS-> this bridge).
#
# Usage: ./run_zenoh_bridge.sh [ros-domain-id]
#   ros-domain-id defaults to 0 (or $ROS_DOMAIN_ID), matching the pixi
#   ros2 env's default unless you've set ROS_DOMAIN_ID there yourself.
set -euo pipefail

DOMAIN_ID="${1:-${ROS_DOMAIN_ID:-0}}"

# --net=host: DDS discovery needs multicast, which doesn't cross a
# container's network namespace (same reason run_microros_agent.sh needs
# it) - required on Linux for this image.
# -l ws/0.0.0.0:10000: the bridge's default zenoh listener is TCP-only;
# browsers need a WebSocket endpoint, so this adds one explicitly on top
# of (not instead of) the default listeners.
exec docker run --init --rm --net=host \
  eclipse/zenoh-bridge-ros2dds \
  -d "$DOMAIN_ID" \
  -l ws/0.0.0.0:10000
