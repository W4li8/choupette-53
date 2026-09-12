#!/usr/bin/env bash
# Builds and flashes one platformio.ini environment to the Maker-ESP32
# board over its onboard CH340 USB-serial adapter.
#
# Usage: ./flash_esp32maker.sh [env]
#   env defaults to platformio.ini's [platformio] default_envs if omitted.
#   Available envs (see platformio.ini): rgb, motors, lidar_point,
#   lidar_matrix, i2c_scan, oled_hello.
#
# Requires /dev/esp32maker, which only exists once the ch341 kernel
# driver + udev rule from ../JetsonCortex/install_ch341_driver have been
# set up on this machine (one-time, persists across reboots/replugs) -
# see that script if this fails with "no such file or directory".
set -euo pipefail

cd "$(dirname "$0")"

if [ ! -e /dev/esp32maker ]; then
  echo "/dev/esp32maker not found." >&2
  echo "Run ../JetsonCortex/install_ch341_driver first (sets up the ch341" >&2
  echo "driver and the /dev/esp32maker udev symlink), then retry." >&2
  exit 1
fi

ENV_ARGS=()
if [ -n "${1:-}" ]; then
  ENV_ARGS=(-e "$1")
fi

pixi run -e platformio pio run "${ENV_ARGS[@]}" -t upload --upload-port /dev/esp32maker
