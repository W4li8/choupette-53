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
  FlushPendingWrite();
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
  FlushPendingWrite();  // shouldn't normally be needed, but defensive
  address_ = address;
  write_buf_.clear();
}

void TwoWire::write(const uint8_t *data, size_t length) {
  write_buf_.insert(write_buf_.end(), data, data + length);
}

void TwoWire::write(uint8_t data) {
  write_buf_.push_back(data);
}

uint8_t TwoWire::endTransmission(bool stop) {
  if (!stop) {
    // Defer: the next requestFrom() will combine this write with the
    // read into a single ioctl(I2C_RDWR) call (real repeated start) -
    // see the class comment in Wire.h for why that's required here.
    write_pending_ = !write_buf_.empty();
    return 0;
  }
  FlushPendingWrite();
  return 0;
}

void TwoWire::FlushPendingWrite() {
  if (write_buf_.empty()) {
    write_pending_ = false;
    return;
  }
  if (fd_ >= 0) {
    i2c_msg msg{};
    msg.addr = address_;
    msg.flags = 0;
    msg.len = static_cast<uint16_t>(write_buf_.size());
    msg.buf = write_buf_.data();

    i2c_rdwr_ioctl_data packets{};
    packets.msgs = &msg;
    packets.nmsgs = 1;
    ioctl(fd_, I2C_RDWR, &packets);
  }
  write_buf_.clear();
  write_pending_ = false;
}

uint8_t TwoWire::requestFrom(uint8_t address, int length, bool /*stop*/) {
  read_buf_.assign(length, 0);
  read_pos_ = 0;
  if (fd_ < 0) {
    read_buf_.clear();
    write_pending_ = false;
    write_buf_.clear();
    return 0;
  }

  i2c_msg read_msg{};
  read_msg.addr = address;
  read_msg.flags = I2C_M_RD;
  read_msg.len = static_cast<uint16_t>(length);
  read_msg.buf = read_buf_.data();

  bool combine = write_pending_ && !write_buf_.empty() && address_ == address;
  int result;
  if (combine) {
    i2c_msg write_msg{};
    write_msg.addr = address_;
    write_msg.flags = 0;
    write_msg.len = static_cast<uint16_t>(write_buf_.size());
    write_msg.buf = write_buf_.data();

    i2c_msg msgs[2] = {write_msg, read_msg};
    i2c_rdwr_ioctl_data packets{};
    packets.msgs = msgs;
    packets.nmsgs = 2;
    result = ioctl(fd_, I2C_RDWR, &packets);
  } else {
    i2c_rdwr_ioctl_data packets{};
    packets.msgs = &read_msg;
    packets.nmsgs = 1;
    result = ioctl(fd_, I2C_RDWR, &packets);
  }
  write_pending_ = false;
  write_buf_.clear();

  if (result < 0) {
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
