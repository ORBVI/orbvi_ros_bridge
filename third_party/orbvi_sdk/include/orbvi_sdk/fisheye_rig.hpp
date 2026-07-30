#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace orbvi_sdk {

// Shared four-camera fisheye rig geometry. The established Panorama* names are
// retained for source and ABI compatibility, but the declarations live in a
// neutral header so spherical panorama and ground-plane BEV can depend on the
// calibration contract without depending on each other's public API.
struct PanoramaCameraIntrinsics {
  std::uint32_t camera_id = 0;
  std::string role;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double xi = 0.0;
  double lambda = 0.0;
  double alpha = 0.0;
  double b = 0.0;
  double c = 0.0;
};

struct PanoramaCameraCalibration {
  PanoramaCameraIntrinsics intrinsics;
  std::array<double, 9> rotation_cam_to_reference = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};
  std::array<double, 3> translation_cam_in_reference_m = {0.0, 0.0, 0.0};
  bool has_full_extrinsics = false;
};

struct PanoramaCalibration {
  std::string calibration_version;
  std::string calibration_hash;
  std::array<double, 3> panorama_origin_reference_m = {0.0, 0.0, 0.0};
  std::vector<PanoramaCameraCalibration> cameras;
};

}  // namespace orbvi_sdk
