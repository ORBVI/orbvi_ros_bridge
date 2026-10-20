#pragma once
#include "orbvi_sdk/frame.hpp"
#include <cstdint>
#include <vector>

namespace orbvi_sdk {
struct RegisteredPoint {
  float x = 0, y = 0, z = 0, intensity = 0;
  std::uint8_t red = 0, green = 0, blue = 0;
};
struct RegisteredCloud {
  std::uint64_t timestamp_ns = 0;
  std::vector<RegisteredPoint> points;
};
// Board Omni-LIVO world (camera_init) coordinates, metres, true image RGB.
// Optional 4VL stream, OLPC v1, uncompressed; at most 1,000,000 points.
// The world frame is not guaranteed Z-up until gravity alignment is enabled.
Result<RegisteredCloud> DecodeRegisteredCloud(const FrameView& frame);
}
