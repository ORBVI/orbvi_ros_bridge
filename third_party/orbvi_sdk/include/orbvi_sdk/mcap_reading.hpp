#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "orbvi_sdk/frame.hpp"

namespace orbvi_sdk {

struct McapReadingOptions {
  std::size_t max_record_bytes = 128U * 1024U * 1024U;
  std::size_t max_catalog_bytes = 16U * 1024U * 1024U;
  std::size_t max_catalog_entries = 4096;
};

struct McapCompressedImage {
  std::uint64_t timestamp_ns = 0;
  std::string frame_id;
  std::string format;
  std::vector<std::uint8_t> data;
};

struct McapImu {
  std::uint64_t timestamp_ns = 0;
  std::string frame_id;
  std::array<double, 4> orientation{};
  std::array<double, 9> orientation_covariance{};
  std::array<double, 3> angular_velocity{};
  std::array<double, 9> angular_velocity_covariance{};
  std::array<double, 3> linear_acceleration{};
  std::array<double, 9> linear_acceleration_covariance{};
};

struct McapMessage {
  std::string topic;
  std::string schema_name;
  std::string message_encoding;
  std::uint32_t sequence = 0;
  std::uint64_t log_time_ns = 0;
  std::uint64_t publish_time_ns = 0;
  std::variant<McapCompressedImage, McapImu, std::string,
               std::vector<std::uint8_t>> data;
};

struct McapReadingInfo {
  std::uint64_t messages = 0;
  bool complete = false;
  std::map<std::string, std::map<std::string, std::string>> metadata;
};

// Inclusive device-time intersection of all selected image views and IMU.
// It only describes observed temporal coverage, not source completeness,
// internal sample gaps, calibration quality or hardware synchronization.
struct McapReplayWindow {
  std::string timestamp_domain;
  std::uint64_t start_ns = 0;
  std::uint64_t end_ns = 0;
  std::uint64_t imu_first_ns = 0;
  std::uint64_t imu_last_ns = 0;
  std::uint64_t imu_preroll_ns = 0;
  bool includes_imu = false;
  bool full_image_coverage = false;
};

class McapReader {
public:
  McapReader();
  ~McapReader();
  McapReader(McapReader&&) noexcept;
  McapReader& operator=(McapReader&&) noexcept;
  McapReader(const McapReader&) = delete;
  McapReader& operator=(const McapReader&) = delete;

  static Result<McapReader> Open(const std::string& path,
                                 const McapReadingOptions& options = {});
  Result<std::optional<McapMessage>> next();
  // Shares the next()/rewind() cursor. Restores recorded SDK frames with
  // original payload, metadata, timestamp and peer contract; does not sleep.
  // Includes recorded FrameSync (StreamId::FrameSync); DecodeFrameSync decodes
  // its original payload. Consumers must dispatch by stream_id, not assume
  // every non-IMU frame is an image. No cross-stream join is performed here.
  // Files without an SDK frame channel report RecordingCapabilityUnavailable.
  // Explicitly recorded MID360 clouds are restored as LidarPointCloud; use
  // DecodeLivoxPointCloudFrame for owned points and timebase/lidar/reserved data.
  // next() validates the standard PointCloud2 CDR and returns its raw bytes,
  // preserving the existing McapMessage variant layout.
  Result<std::optional<OwnedFrame>> nextSdkFrame();
  Result<void> rewind();
  McapReadingInfo info() const;
  // Consume through EOF first (CRC/completion validation); does not scan,
  // rewind or filter frames. Old/unsupported windows report unavailable.
  Result<McapReplayWindow> replayWindow() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}
