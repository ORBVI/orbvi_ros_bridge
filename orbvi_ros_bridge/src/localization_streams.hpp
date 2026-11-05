#pragma once

#include <algorithm>
#include <vector>

#include "orbvi_sdk/status.hpp"

namespace orbvi_ros_bridge {

inline std::vector<orbvi_sdk::StreamId> ResolveLocalizationStreams(
    const std::vector<orbvi_sdk::StreamId>& requested) {
  std::vector<orbvi_sdk::StreamId> resolved;
  const auto append = [&resolved](orbvi_sdk::StreamId stream) {
    if (std::find(resolved.begin(), resolved.end(), stream) == resolved.end()) {
      resolved.push_back(stream);
    }
  };
  for (const auto stream : requested) {
    // localization/vio select both pose outputs. The legacy mixed stream is
    // accepted as a configuration alias, but must never reach live subscribe.
    if (stream == orbvi_sdk::StreamId::VioPose) {
      append(orbvi_sdk::StreamId::OptimizedPose);
      append(orbvi_sdk::StreamId::PredictedPose);
    } else {
      append(stream);
    }
  }
  return resolved;
}

}  // namespace orbvi_ros_bridge
