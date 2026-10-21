#pragma once

#include "orbvi_sdk/frame.hpp"
#include "orbvi_camera_contract/ground_projection.hpp"

namespace orbvi_sdk {
using RobotGroundProjection = orbvi_camera_contract::RobotGroundProjection;

// Reads the exact board GPU projection snapshot; never infers it from frame_id.
// Legacy display BEV deliberately does not satisfy the robot contract.
inline Result<RobotGroundProjection> ReadBoardRobotProjection(const FrameView& frame) {
  const auto fail=[]() -> Result<RobotGroundProjection> {
    return SdkError{SdkErrorCode::ProtocolError,"Missing or invalid board GPU robot IPM geometry"};
  };
  if (frame.stream_id != StreamId::BevDisplay || !frame.metadata) return fail();
  const auto& m=*frame.metadata;
  const auto projection=m.find("projection"), gpu=m.find("gpu_projected"),
      cpu=m.find("cpu_fallback_used"), geometry=m.find("robot_projection_v1");
  RobotGroundProjection p;
  if (projection==m.end() || projection->second!="robot_ground_plane_ipm" ||
      gpu==m.end() || gpu->second!="true" || cpu==m.end() || cpu->second!="false" ||
      geometry==m.end() || !orbvi_camera_contract::ParseRobotGroundProjection(geometry->second,&p) ||
      frame.frame_id!=p.parent_frame || frame.calibration_hash.empty()) return fail();
  return p;
}
}  // namespace orbvi_sdk
