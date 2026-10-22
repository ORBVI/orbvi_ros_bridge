#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "orbvi_sdk/fisheye_rig.hpp"
#include "orbvi_sdk/frame.hpp"
#include "orbvi_sdk/lidar_calibration.hpp"

namespace orbvi_sdk {
struct CalibrationStatus;
}

namespace orbvi_ros_bridge {

// Device-fixed SDK ROS_BODY origin and axes; not a robot mounting frame.
inline constexpr char kDefaultDeviceFrame[] = "orbvi_link";

// T_parent_child maps child coordinates into parent; translations are metres.
struct TransformRecord {
  std::string parent;
  std::string child;
  std::array<double, 3> translation{};
  std::array<double, 4> quaternion{{0, 0, 0, 1}}; // xyzw
};
struct SensorTransforms {
  std::vector<TransformRecord> transforms;
  std::vector<std::string> diagnostics;
  std::string camera_hash;
  std::string lidar_hash;
};
std::string RawCameraFrameId(const std::string &camera_id);
bool ValidTransform(const TransformRecord &transform);
SensorTransforms
BuildSensorTransforms(const orbvi_sdk::LidarCalibration *lidar,
                      const orbvi_sdk::PanoramaCalibration *rig,
                      const std::string &depth_frame = kDefaultDeviceFrame);
SensorTransforms
LoadSensorTransforms(const orbvi_sdk::CalibrationStatus &calibration,
                     const std::string &depth_frame = kDefaultDeviceFrame);
std::string ExtrinsicsJson(const SensorTransforms &snapshot);

// One bounded gate per optimized pose stream. Missing producer epochs never
// authorize a backwards timestamp. A forward-time sequence reset is
// recoverable.
class PoseTransformGate {
public:
  bool Accept(const orbvi_sdk::FrameView &frame,
              bool *producer_changed = nullptr,
              std::string *rejection_reason = nullptr);

private:
  std::uint64_t timestamp_ = 0;
  std::uint64_t sequence_ = 0;
  std::string epoch_;
  std::vector<std::string> retired_epochs_;
};
} // namespace orbvi_ros_bridge
