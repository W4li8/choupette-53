// Minimal Arduino TwoWire-compatible shim backed by Linux i2c-dev, just
// enough for the two vendored drivers this package reuses unmodified
// (src/apps/dfrobot_matrix_lidar.inc and third_party/pololu_vl53l0x) to
// run unchanged on the Jetson: begin(), beginTransmission(),
// write(buf, len), write(byte), endTransmission()/(bool),
// requestFrom(addr, len, bool), read(). Not a general Wire.h
// replacement.
//
// endTransmission(false) means a real repeated start: this Tegra I2C
// controller (/dev/i2c-7) was found to NOT reliably support a register
// read done as two independent ioctl(I2C_RDWR) calls (write-register-
// pointer, then a separate read) - a raw register read (e.g. VL53L0X's
// fixed model-ID register) would silently come back wrong that way, but
// reads correctly when the write+read are issued as ONE combined
// i2c_rdwr call. endTransmission(false) defers the buffered write so
// the next requestFrom() can combine them; endTransmission(true)
// (the default, and what dfrobot_matrix_lidar.inc always uses) sends
// immediately, unchanged from before.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class TwoWire {
 public:
  explicit TwoWire(const char *devicePath);
  ~TwoWire();

  void begin();
  void beginTransmission(uint8_t address);
  void write(const uint8_t *data, size_t length);
  void write(uint8_t data);
  uint8_t endTransmission(bool stop = true);
  uint8_t requestFrom(uint8_t address, int length, bool stop = true);
  int read();

 private:
  void FlushPendingWrite();

  std::string device_path_;
  int fd_ = -1;
  uint8_t address_ = 0;
  std::vector<uint8_t> write_buf_;
  bool write_pending_ = false;  // true between endTransmission(false) and the next requestFrom()
  std::vector<uint8_t> read_buf_;
  size_t read_pos_ = 0;
};

// The I2C bus wired to the Jetson's header I2C1 (physical pins 3/5) -
// confirmed via i2cdetect to be /dev/i2c-7 on this Jetson, NOT
// /dev/i2c-1 despite the device-tree alias literally named "i2c1"
// pointing elsewhere (see project memory: don't trust that alias name
// for header pin mapping on this board).
extern TwoWire Wire;
