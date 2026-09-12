// Jetson-side ROS2 node for the DFRobot Gravity 8x8 Matrix ToF sensor
// (VL53L7CX + onboard RP2040, SKU SEN0628). Reuses the vendored I2C
// protocol driver from the ESP32 firmware unmodified (see
// dfrobot_matrix_lidar.inc below) via the Arduino/Wire compatibility
// shims in arduino_compat/, rather than re-deriving the packet protocol
// in a second language - see src/apps/dfrobot_matrix_lidar.inc for the
// protocol details and src/apps/lidar_matrix_app.inc for the ESP32
// firmware this Jetson node replaces (kept in place as a fallback).
#include <cmath>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "arduino_compat/Arduino.h"
#include "arduino_compat/Wire.h"

// Found via the include directory CMakeLists.txt points at
// ../../../../src/apps - not copied or symlinked, so any future firmware
// change to the protocol logic applies here too.
#include "dfrobot_matrix_lidar.inc"

namespace {

constexpr int kMatrixSize = 8;
constexpr int kMatrixZones = kMatrixSize * kMatrixSize;

// VL53L7CX 60x60deg FOV in 8x8 mode (ST datasheet) / 8 zones per axis.
// This is a linear-grid approximation, not ST's (unpublished in an
// easily citable form) lens-corrected per-zone table - close enough for
// a coarse safety-stop distance check, but NOT verified against which
// physical direction zone (row 0, col 0) of the DFRobot buffer actually
// points at: the VL53L7CX's lens flips the image (datasheet: "zone 0 in
// the bottom left of the SPAD array is illuminated by a target located
// at the top right-hand side of the scene"), and it's unconfirmed
// whether DFRobot's RP2040 firmware already un-flips this before
// handing zones over I2C. Verify empirically (point a known target to
// one side, check which array index responds) before trusting this for
// anything beyond a coarse "something is close" signal.
constexpr double kFovDeg = 60.0;
constexpr double kZonePitchDeg = kFovDeg / kMatrixSize;  // 7.5 deg/zone

double ZoneAngleDeg(int index) {
  return (index - (kMatrixSize - 1) / 2.0) * kZonePitchDeg;
}

}  // namespace

class LidarMatrixNode : public rclcpp::Node {
 public:
  LidarMatrixNode() : Node("lidar_matrix_node"), sensor_(0x33) {
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/i2c_0x33/points", 10);
    // VL53L7CX chip caps at 15Hz in 8x8 mode (ST datasheet); the
    // DFRobot RP2040's request/response protocol adds its own overhead
    // on top, so this timer period is a ceiling, not a promise.
    timer_ = create_wall_timer(std::chrono::milliseconds(66),
                                std::bind(&LidarMatrixNode::Tick, this));
    TryInit();
  }

 private:
  void TryInit() {
    ready_ = (sensor_.begin() == 0) && (sensor_.setRangingMode(eMatrix_8X8) == 0);
    if (ready_) {
      RCLCPP_INFO(get_logger(), "sensor ready at 0x33");
    } else {
      RCLCPP_WARN(get_logger(), "init failed, retrying...");
    }
  }

  void Tick() {
    if (!ready_) {
      TryInit();
      return;
    }

    uint16_t buf[kMatrixZones];
    if (sensor_.getAllData(buf) != 0) {
      RCLCPP_WARN(get_logger(), "getAllData failed, reinitializing...");
      ready_ = false;
      return;
    }
    PublishCloud(buf);
  }

  // Per-point XYZ is computed from range * the zone's own beam
  // direction (REP-103: x forward, y left, z up), in this sensor's own
  // frame - the sensor's mounting orientation on the robot is a
  // separate concern, left to a static transform (see pixi.toml's
  // lidar_matrix_tf task) rather than baked in here.
  void PublishCloud(const uint16_t *buf) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = "lidar_i2c_0x33";
    cloud.height = 1;
    cloud.width = kMatrixZones;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(kMatrixZones);

    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");

    // Row-major, matching the sensor's own zone layout: buf[y*8+x].
    for (int y = 0; y < kMatrixSize; y++) {
      for (int x = 0; x < kMatrixSize; x++, ++ix, ++iy, ++iz) {
        float range_m = buf[y * kMatrixSize + x] / 1000.0f;
        double az = ZoneAngleDeg(x) * M_PI / 180.0;
        double el = ZoneAngleDeg(y) * M_PI / 180.0;
        *ix = static_cast<float>(range_m * std::cos(el) * std::cos(az));
        *iy = static_cast<float>(range_m * std::cos(el) * std::sin(az));
        *iz = static_cast<float>(range_m * std::sin(el));
      }
    }

    pub_->publish(cloud);
  }

  DFRobot_MatrixLidar_I2C sensor_;
  bool ready_ = false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  Wire.begin();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarMatrixNode>());
  rclcpp::shutdown();
  return 0;
}
