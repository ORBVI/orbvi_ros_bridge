#pragma once

#include <cmath>
#include "orbvi_sdk/livox.hpp"

namespace orbvi_ros_bridge {

inline bool ValidLidarImuParameters(const orbvi_sdk::LidarImuParameters* parameters) {
  if (!parameters ||
      (parameters->linear_acceleration_unit != orbvi_sdk::LinearAccelerationUnit::StandardGravity &&
       parameters->linear_acceleration_unit != orbvi_sdk::LinearAccelerationUnit::MetersPerSecondSquared))
    return false;
  const double scale = parameters->linear_acceleration_scale_to_mps2;
  return std::isfinite(scale) && scale > 0 && std::isfinite(scale * scale);
}

template <typename ImuMessage>
bool ConvertLidarImuToRosUnits(const orbvi_sdk::LidarImuParameters* parameters,
                              ImuMessage* message) {
  if (!message || !ValidLidarImuParameters(parameters)) return false;
  const double scale = parameters->linear_acceleration_scale_to_mps2;
  message->linear_acceleration.x *= scale;
  message->linear_acceleration.y *= scale;
  message->linear_acceleration.z *= scale;
  // sensor_msgs/Imu uses a leading -1 to mark an unavailable estimate.
  if (message->linear_acceleration_covariance[0] != -1.0) {
    for (auto& value : message->linear_acceleration_covariance) value *= scale * scale;
  }
  return true;
}
}  // namespace orbvi_ros_bridge
