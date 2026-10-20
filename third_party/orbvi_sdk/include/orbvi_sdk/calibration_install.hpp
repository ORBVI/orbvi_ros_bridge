#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace orbvi_sdk {

inline constexpr std::size_t kMaxCalibrationInstallPackageBytes = 8U * 1024U * 1024U;

struct CalibrationFileCheck {
  std::string path;
  std::string expected_sha256;
  std::string actual_sha256;
  bool matches = false;
  std::string status;
};

// The effective collection may include a previously installed camera package
// plus a VINS supplement. Its hash need not equal the last uploaded package.
struct InstalledCalibration {
  std::string calibration_status;
  std::string calibration_version;
  std::string calibration_hash;
  bool installed = false;
  std::string state;
  std::string effective_bundle_hash;
  std::string manifest_json;
  // Optional external-tool master document after device-side hash validation.
  // Geometry/schema ownership remains with the external calibration tool.
  std::string master_json;
  std::string vins_state;
  bool files_verified = false;
  std::vector<CalibrationFileCheck> file_checks;
  std::string raw_json;
};

struct CalibrationInstallAcceptance {
  bool accepted = false;
  std::string operation_id;
  std::string state;
  std::string bundle_hash;
  std::string message;
  std::string raw_json;
};

struct CalibrationInstallStatus {
  std::string operation_id;
  std::string state;
  bool terminal = false;
  bool success = false;
  std::string bundle_hash;
  std::string effective_bundle_hash;
  std::string message;
  std::string raw_json;
};

struct CalibrationInstallWaitOptions {
  // One deadline covers upload, service recovery, polling and final readback.
  std::uint32_t timeout_ms = 120000;
  std::uint32_t poll_interval_ms = 500;
};

struct CalibrationInstallProgress {
  std::string operation_id;
  // accepted / applying / verifying / succeeded / readback_verified, etc.
  std::string state;
  std::string message;
};
using CalibrationInstallProgressCallback = std::function<void(const CalibrationInstallProgress&)>;

struct CalibrationInstallResult {
  std::string operation_id;
  CalibrationInstallStatus status;
  InstalledCalibration installed;
};

}  // namespace orbvi_sdk
