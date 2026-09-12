# Vendored: Pololu VL53L0X Arduino library

`VL53L0X.h`/`VL53L0X.cpp` are copied unmodified from
[pololu/vl53l0x-arduino](https://github.com/pololu/vl53l0x-arduino)
(fetched via PlatformIO's `pololu/VL53L0X@^1.3.1` for the ESP32 build -
see `../../../../platformio.ini`'s `lidar_point` env), so `lidar_point_node.cpp`
can reuse the exact same VL53L0X driver logic the ESP32 firmware uses
(`../../../../src/apps/lidar_point_app.inc`) instead of re-deriving it.

Only Arduino API surface used: `millis()`, and `Wire`/`TwoWire`'s
`beginTransmission`/`write` (both the single-byte and buffer+length
overloads)/`endTransmission`/`requestFrom`/`read` - all covered by the
shims in `../../include/arduino_compat/`.

MIT-licensed by Pololu Corporation (some logic derived from ST's VL53L0X
API, also permissively licensed) - see `LICENSE.txt` in this directory.
