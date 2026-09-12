# Vendored: Pololu VL53L0X Arduino library

`VL53L0X.h`/`VL53L0X.cpp` are copied from
[pololu/vl53l0x-arduino](https://github.com/pololu/vl53l0x-arduino)
(fetched via PlatformIO's `pololu/VL53L0X@^1.3.1` for the ESP32 build -
see `../../../../platformio.ini`'s `lidar_point` env), so `lidar_point_node.cpp`
can reuse the exact same VL53L0X driver logic the ESP32 firmware uses
(`../../../../src/apps/lidar_point_app.inc`) instead of re-deriving it.

**One deliberate deviation from upstream:** the four register-read
helpers (`readReg`, `readReg16Bit`, `readReg32Bit`, `readMulti`) call
`bus->endTransmission(false)` instead of upstream's `endTransmission()`.
This Jetson's I2C1 controller (`/dev/i2c-7`) doesn't reliably support a
register read framed as two independent `ioctl(I2C_RDWR)` calls - even
the sensor's fixed model-ID register came back wrong that way, though
writes and a *combined* write+read both worked fine. `endTransmission(false)`
requests a real repeated-start from the `TwoWire` shim (see
`../../include/arduino_compat/Wire.h`), which combines the pending
register-pointer write with the following `requestFrom()` into one
`ioctl(I2C_RDWR)` call. Every other call site (all the write-only
helpers) is untouched.

Only Arduino API surface used: `millis()`, and `Wire`/`TwoWire`'s
`beginTransmission`/`write` (both the single-byte and buffer+length
overloads)/`endTransmission`/`requestFrom`/`read` - all covered by the
shims in `../../include/arduino_compat/`.

MIT-licensed by Pololu Corporation (some logic derived from ST's VL53L0X
API, also permissively licensed) - see `LICENSE.txt` in this directory.
