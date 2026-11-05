#pragma once

#include <algorithm>
#include <string>
#include "imu_units.hpp"
#include "orbvi_sdk/device_info.hpp"

namespace orbvi_ros_bridge {

// An absent legacy capability manifest is inconclusive: let subscribe decide.
// Registered clouds are optional even on devices with a LiDAR.
inline std::string StreamUnavailableReason(
    orbvi_sdk::StreamId stream, const orbvi_sdk::DeviceInfo* device,
    const orbvi_sdk::LidarImuParameters* lidar_imu) {
  if (stream == orbvi_sdk::StreamId::LidarImu && !ValidLidarImuParameters(lidar_imu))
    return "missing or unsupported LiDAR IMU units; refusing to publish non-SI acceleration";
  if (stream == orbvi_sdk::StreamId::RegisteredCloud && device) {
    const std::string name = orbvi_sdk::ToString(stream);
    const auto contains = [&name](const auto& names) {
      return std::find(names.begin(), names.end(), name) != names.end();
    };
    if (contains(device->unsupported_streams) ||
        (!device->supported_streams.empty() && !contains(device->supported_streams)))
      return "device does not advertise registered_cloud_stream";
  }
  return {};
}

}  // namespace orbvi_ros_bridge
