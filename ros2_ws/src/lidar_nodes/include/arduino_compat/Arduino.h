// Minimal Arduino-compatibility shim: just enough for
// src/apps/dfrobot_matrix_lidar.inc (vendored unmodified from the ESP32
// firmware, see ../../../../src/apps/dfrobot_matrix_lidar.inc) to
// compile against on Linux. Not a general Arduino compatibility layer -
// only the handful of calls that file actually makes.
#pragma once

#include <chrono>
#include <cstdint>
#include <thread>

// Arduino core typedef (e.g. used by third_party/pololu_vl53l0x's
// VL53L0X.h), not a standard C++ type.
typedef bool boolean;

inline void delay(unsigned long ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline unsigned long millis() {
  using namespace std::chrono;
  static const auto start = steady_clock::now();
  return static_cast<unsigned long>(duration_cast<milliseconds>(steady_clock::now() - start).count());
}

inline void yield() {
  std::this_thread::yield();
}
