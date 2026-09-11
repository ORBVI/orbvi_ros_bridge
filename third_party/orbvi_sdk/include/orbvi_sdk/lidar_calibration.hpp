#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "orbvi_sdk/status.hpp"

namespace orbvi_sdk {

struct CalibrationStatus;

// Factory calibration for the rigid LiDAR/camera mount. The transform is
// camera <- LiDAR (Rcl * p_lidar + Pcl), with translation in metres. Camera
// intrinsics and camera-to-rig poses remain in camera_config.yaml; this type
// only represents the separate FAST-Calib LiDAR files.
struct LidarCameraExtrinsic {
  std::string source_frame;
  std::string target_frame;
  std::uint32_t camera_index = 0;
  std::uint32_t camera_id = 0;
  std::string role;
  std::string convention;
  std::array<double, 9> rotation_matrix_3x3 = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};
  std::array<double, 3> translation_m = {0.0, 0.0, 0.0};
};

struct LidarImuExtrinsic {
  std::string sensor_model;
  std::string parameter_source;
  std::string source_frame;
  std::string target_frame;
  std::string translation_unit;
  std::string source_file;
  std::array<double, 9> rotation_matrix{};
  std::array<double, 3> translation_m{};
};

struct LidarCalibration {
  std::string calibration_version;
  // The camera bundle hash is retained separately so a LiDAR update does not
  // change existing CalibrationStatus::calibration_hash cache semantics.
  std::string camera_calibration_hash;
  std::string calibration_hash;
  std::string source;
  std::string status;
  std::string hash_status;
  std::string blocked_reason;
  std::string lidar_model;
  bool lidar_imu_extrinsics_available = false;
  std::vector<LidarCameraExtrinsic> camera_extrinsics;
  std::optional<LidarImuExtrinsic> imu_extrinsic;
};

Result<LidarCalibration> ParseLidarCalibrationJson(const std::string& calibration_json);
Result<LidarCalibration> MakeLidarCalibration(const CalibrationStatus& calibration_status);

}  // namespace orbvi_sdk
