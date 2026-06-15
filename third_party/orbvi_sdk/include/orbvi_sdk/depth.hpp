#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "orbvi_sdk/frame.hpp"

namespace orbvi_sdk {

struct CalibrationStatus;

enum class DepthPixelFormat {
  Float32Meters,
  Uint16Millimeters,
};

enum class DepthInvalidPolicy {
  NaN,
  Zero,
  MaxRange,
};

struct DepthTileCalibration {
  std::uint32_t tile_index = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  double fx_px = 0.0;
  double fy_px = 0.0;
  double baseline_m = 0.0;
  double cx_px = 0.0;
  double cy_px = 0.0;
  std::array<double, 9> rotation_world = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> origin_translation_m = {0.0, 0.0, 0.0};
  bool has_world_transform = false;
};

struct DepthCalibration {
  std::string calibration_version;
  std::string calibration_hash;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t tile_count = 0;
  std::uint32_t tile_width = 0;
  std::uint32_t tile_height = 0;
  std::vector<DepthTileCalibration> tiles;
};

struct DepthGenerationOptions {
  DepthPixelFormat output_format = DepthPixelFormat::Float32Meters;
  DepthInvalidPolicy invalid_policy = DepthInvalidPolicy::NaN;
  double min_disparity_px = 0.01;
  double max_depth_m = 50.0;
  bool include_source_disparity = false;
  bool preserve_disparity_metadata = true;
};

struct DepthMapView {
  DepthPixelFormat pixel_format = DepthPixelFormat::Float32Meters;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t stride = 0;
  std::uint64_t timestamp_ns = 0;
  std::string frame_id;
  std::string calibration_version;
  std::string calibration_hash;
  const std::uint8_t* data = nullptr;
  std::size_t data_size = 0;
  const MetadataMap* metadata = nullptr;
};

struct DepthMap {
  DepthPixelFormat pixel_format = DepthPixelFormat::Float32Meters;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t stride = 0;
  std::uint64_t timestamp_ns = 0;
  std::string frame_id;
  std::string calibration_version;
  std::string calibration_hash;
  std::vector<std::uint8_t> data;
  MetadataMap metadata;

  DepthMapView view() const;
};

enum class PointCloudFrame {
  TileOptical,
  CalibrationWorld,
  RosBody,
};

struct PointCloudGenerationOptions {
  double min_disparity_px = 0.5;
  double min_depth_m = 0.1;
  double max_depth_m = 20.0;
  double max_disparity_jump_px = 2.0;
  std::uint32_t sample_stride = 4;
  PointCloudFrame output_frame = PointCloudFrame::RosBody;
  std::string frame_id;
};

struct PointCloudPoint {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
};

struct PointCloud {
  std::uint64_t timestamp_ns = 0;
  std::string frame_id;
  std::string calibration_version;
  std::string calibration_hash;
  std::vector<PointCloudPoint> points;
  MetadataMap metadata;
};

struct DepthSubscriptionStats {
  TransportMode transport_mode = TransportMode::SampleEndpoint;
  double observed_rate_hz = 0.0;
  double depth_latency_ms = 0.0;
  double max_depth_latency_ms = 0.0;
  std::uint64_t delivered_depth_maps = 0;
  std::uint64_t dropped_frames = 0;
  std::uint64_t disparity_parse_errors = 0;
  std::uint64_t depth_failures = 0;
  std::uint64_t calibration_refresh_count = 0;
  std::string blocked_reason;
};

struct DepthFrameDelivery {
  DepthMap depth;
  std::optional<OwnedFrame> source_disparity;
  DepthSubscriptionStats stats;
};

using DepthFrameCallback = std::function<void(const DepthFrameDelivery&)>;

struct DepthSubscribeOptions {
  DepthGenerationOptions depth;
  std::size_t max_receive_queue_depth = 32;
  std::uint32_t first_frame_timeout_ms = 5000;
  bool require_streaming_transport = true;
  bool allow_sample_endpoint_fallback = false;
};

Result<DepthCalibration> ParseDepthCalibrationJson(const std::string& calibration_json);
Result<DepthCalibration> MakeDepthCalibration(const CalibrationStatus& calibration_status);
bool IsDisparityFrame(const FrameView& frame);
Result<DepthMap> GenerateDepthMap(
    const FrameView& disparity_frame,
    const DepthCalibration& calibration,
    const DepthGenerationOptions& options = {});
Result<DepthMap> GenerateDepthMap(
    const OwnedFrame& disparity_frame,
    const DepthCalibration& calibration,
    const DepthGenerationOptions& options = {});
Result<PointCloud> GeneratePointCloud(
    const FrameView& disparity_frame,
    const DepthCalibration& calibration,
    const PointCloudGenerationOptions& options = {});
Result<PointCloud> GeneratePointCloud(
    const OwnedFrame& disparity_frame,
    const DepthCalibration& calibration,
    const PointCloudGenerationOptions& options = {});

const char* ToString(DepthPixelFormat format);
const char* ToString(DepthInvalidPolicy policy);
const char* ToString(PointCloudFrame frame);

}  // namespace orbvi_sdk
