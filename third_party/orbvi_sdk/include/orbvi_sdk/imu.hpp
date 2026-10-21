#pragma once

#include <array>
#include "orbvi_sdk/frame.hpp"

namespace orbvi_sdk {

// Device main IMU only. Values remain in the source sensor frame: quaternion
// x/y/z/w, angular velocity rad/s, linear acceleration m/s^2. No filtering,
// rotation, integration or quaternion normalization is performed.
struct ImuSample {
  std::uint64_t timestamp_ns = 0;
  TimestampDomain timestamp_domain = TimestampDomain::Unknown;
  std::string frame_id;
  std::array<double, 4> orientation{};
  std::array<double, 3> angular_velocity{};
  std::array<double, 3> linear_acceleration{};
  // Row-major 3x3; preserve -1 at element 0 (unavailable measurement) and
  // all-zero covariance (unknown uncertainty), without fabricating estimates.
  std::array<double, 9> orientation_covariance{};
  std::array<double, 9> angular_velocity_covariance{};
  std::array<double, 9> linear_acceleration_covariance{};
};

// Validates stream/format/compression, exact payload length, selected peer's
// layout, SI units and finite values. The frame must already have passed wire
// CRC/envelope validation. Unknown time domains remain unknown; this function
// does not establish synchronization, sensor completeness or calibration.
// The result owns its values and frame_id; input bytes/metadata are unchanged.
Result<ImuSample> ParseImuSample(const FrameView& frame);

}  // namespace orbvi_sdk
