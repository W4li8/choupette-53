#include "arduino_compat/Wire.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdio>

#ifndef LIDAR_MATRIX_I2C_DEVICE
#define LIDAR_MATRIX_I2C_DEVICE "/dev/i2c-7"
#endif

TwoWire Wire(LIDAR_MATRIX_I2C_DEVICE);

TwoWire::TwoWire(const char *devicePath) : device_path_(devicePath) {}

TwoWire::~TwoWire() {
  if (fd_ >= 0) {
    close(fd_);
  }
}

void TwoWire::begin() {
  if (fd_ >= 0) {
    return;
  }
  fd_ = open(device_path_.c_str(), O_RDWR);
  if (fd_ < 0) {
    perror(("TwoWire::begin: open " + device_path_).c_str());
  }
}

void TwoWire::beginTransmission(uint8_t address) {
  address_ = address;
  write_buf_.clear();
}

void TwoWire::write(const uint8_t *data, size_t length) {
  write_buf_.insert(write_buf_.end(), data, data + length);
}

void TwoWire::write(uint8_t data) {
  write_buf_.push_back(data);
}

uint8_t TwoWire::endTransmission(bool /*stop*/) {
  if (write_buf_.empty()) {
    return 0;
  }
  if (fd_ < 0) {
    write_buf_.clear();
    return 1;
  }

  i2c_msg msg{};
  msg.addr = address_;
  msg.flags = 0;
  msg.len = static_cast<uint16_t>(write_buf_.size());
  msg.buf = write_buf_.data();

  i2c_rdwr_ioctl_data packets{};
  packets.msgs = &msg;
  packets.nmsgs = 1;

  uint8_t result = (ioctl(fd_, I2C_RDWR, &packets) < 0) ? 1 : 0;
  write_buf_.clear();
  return result;
}

uint8_t TwoWire::requestFrom(uint8_t address, int length, bool /*stop*/) {
  read_buf_.assign(length, 0);
  read_pos_ = 0;
  if (fd_ < 0) {
    read_buf_.clear();
    return 0;
  }

  i2c_msg msg{};
  msg.addr = address;
  msg.flags = I2C_M_RD;
  msg.len = static_cast<uint16_t>(length);
  msg.buf = read_buf_.data();

  i2c_rdwr_ioctl_data packets{};
  packets.msgs = &msg;
  packets.nmsgs = 1;

  if (ioctl(fd_, I2C_RDWR, &packets) < 0) {
    read_buf_.clear();
    return 0;
  }
  return static_cast<uint8_t>(read_buf_.size());
}

int TwoWire::read() {
  if (read_pos_ >= read_buf_.size()) {
    return -1;
  }
  return read_buf_[read_pos_++];
}
