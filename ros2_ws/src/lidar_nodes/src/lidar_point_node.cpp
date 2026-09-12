// Jetson-side ROS2 node for the VL53L0X single-point ToF sensor.
// Reuses the vendored Pololu VL53L0X Arduino library unmodified (see
// third_party/pololu_vl53l0x/) via the same Arduino/Wire compatibility
// shims as lidar_matrix_node.cpp, rather than re-deriving the sensor's
// init/calibration sequence in a second language - see
// src/apps/lidar_point_app.inc for the protocol details and the ESP32
// firmware this Jetson node replaces (kept in place as a fallback).
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "arduino_compat/Arduino.h"
#include "arduino_compat/Wire.h"
#include "VL53L0X.h"

namespace {
// Matches the ESP32 firmware's pointSensors[i].setTimeout(500).
constexpr uint16_t kIoTimeoutMs = 500;
// ~30Hz, matching the ESP32 firmware's measured achieved rate.
constexpr int kReadPeriodMs = 33;
}  // namespace

class LidarPointNode : public rclcpp::Node {
 public:
  LidarPointNode() : Node("lidar_point_node") {
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/i2c_0x29/points", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(kReadPeriodMs),
                                std::bind(&LidarPointNode::Tick, this));
    TryInit();
  }

 private:
  void TryInit() {
    sensor_.setTimeout(kIoTimeoutMs);
    ready_ = sensor_.init();
    if (ready_) {
      sensor_.startContinuous();
      RCLCPP_INFO(get_logger(), "sensor ready at 0x29");
    } else {
      RCLCPP_WARN(get_logger(), "init failed, retrying...");
    }
  }

  void Tick() {
    if (!ready_) {
      TryInit();
      return;
    }

    uint16_t distance_mm = sensor_.readRangeContinuousMillimeters();

    // Same instinct as the ESP32 firmware: a timeout means we've lost
    // the sensor, so re-run init from scratch rather than trust it's
    // still in a good state.
    if (sensor_.timeoutOccurred()) {
      RCLCPP_WARN(get_logger(), "timeout, reinitializing...");
      ready_ = false;
      return;
    }

    // VL53L0X hardware quirk (see lidar_point_app.inc): ~8190mm means
    // "no target in range", not an error - published as a normal
    // point. Downstream (safety-stop node) decides what counts as
    // "too close", not this driver node.
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = "lidar_i2c_0x29";
    cloud.height = 1;
    cloud.width = 1;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);

    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
    // REP-103: x forward - this sensor has exactly one beam, along its
    // own frame's +X. Mounting orientation on the robot is a separate
    // concern (see pixi.toml's lidar_point_tf task), not baked in here.
    *ix = distance_mm / 1000.0f;
    *iy = 0.0f;
    *iz = 0.0f;

    pub_->publish(cloud);
  }

  VL53L0X sensor_;
  bool ready_ = false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  Wire.begin();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPointNode>());
  rclcpp::shutdown();
  return 0;
}
