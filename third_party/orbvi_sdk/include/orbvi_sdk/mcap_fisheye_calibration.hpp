#pragma once

#include "orbvi_sdk/fisheye_calibration.hpp"
#include "orbvi_sdk/mcap_reading.hpp"

namespace orbvi_sdk {

struct McapFisheyeCalibrationOptions {
  std::string mcap_path;
  std::string dataset_directory;
  FisheyeCalibrationOptions calibration;
  std::array<std::string, 4> image_ids;
  TimestampDomain image_timestamp_domain = TimestampDomain::Unknown;
  std::uint64_t max_timestamp_skew_ns = 0;
  std::uint64_t max_dataset_bytes = 8ULL * 1024 * 1024 * 1024;
  std::uint64_t max_synchronized_sets = 100000;
  std::size_t max_pending_images = 256;
  std::size_t max_pending_bytes = 128U * 1024 * 1024;
  std::uint64_t max_image_pixels = 32U * 1024 * 1024;
  McapReadingOptions reading;
};

struct McapFisheyeInputReport {
  std::string source_mcap;
  std::string source_sha256;
  std::string groups_sha256;
  std::map<std::string, std::string> recording_statistics;
  std::string camera_topic_scheme;
  FisheyeCalibrationOptions calibration;
  std::array<std::string, 4> image_ids;
  TimestampDomain image_timestamp_domain = TimestampDomain::Unknown;
  std::array<std::uint64_t, 4> image_messages{};
  std::uint64_t synchronized_sets = 0;
  std::uint64_t unmatched_images = 0;
  std::uint64_t ignored_messages = 0;
  std::uint64_t maximum_matched_skew_ns = 0;
  std::uint64_t max_timestamp_skew_ns = 0;
  std::uint64_t dataset_bytes = 0;
};

struct McapFisheyeCalibrationResult {
  FisheyeCalibrationResult calibration;
  McapFisheyeInputReport input;
};

Result<McapFisheyeInputReport> PrepareFisheyeDatasetFromMcap(
    const McapFisheyeCalibrationOptions& options);
Result<McapFisheyeCalibrationResult> CalibrateFisheyeFromMcap(
    const McapFisheyeCalibrationOptions& options);
Result<void> SaveFisheyeCalibration(const McapFisheyeCalibrationResult& result,
                                   const std::string& output_directory);

}
