#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "orbvi_sdk/frame.hpp"
#include "orbvi_sdk/image.hpp"

namespace orbvi_sdk {

struct CalibrationStatus;

enum class PanoramaBlendMode {
  PrimaryOnly,
  Feather,
  MultiBand,
};

enum class PanoramaSeamMode {
  Fixed,
  DynamicProgramming,
};

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
};

struct PanoramaCalibration {
  std::string calibration_version;
  std::string calibration_hash;
  std::vector<PanoramaCameraCalibration> cameras;
};

struct PanoramaSeamAvoidanceMaskView {
  std::string camera_id;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t stride = 0;
  const std::uint8_t* data = nullptr;
  std::size_t data_size = 0;
};

struct PanoramaStitchOptions {
  std::uint32_t width = 2048;
  std::uint32_t height = 1024;
  double fov_half_deg = 95.0;
  std::uint32_t seam_blend_px = 32;
  PanoramaBlendMode blend_mode = PanoramaBlendMode::Feather;
  PanoramaSeamMode seam_mode = PanoramaSeamMode::Fixed;
  std::uint32_t dp_seam_band_px = 96;
  double dp_seam_smoothness = 8.0;
  bool photometric_align = true;
  bool seam_ghost_suppression = false;
  double seam_ghost_threshold = 80.0;
  std::vector<PanoramaSeamAvoidanceMaskView> seam_avoidance_masks;
  double seam_avoidance_penalty = 220.0;
};

struct PanoramaImageView {
  PixelFormat pixel_format = PixelFormat::Bgr888;
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

struct OwnedPanoramaImage {
  PixelFormat pixel_format = PixelFormat::Bgr888;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t stride = 0;
  std::uint64_t timestamp_ns = 0;
  std::string frame_id;
  std::string calibration_version;
  std::string calibration_hash;
  std::vector<std::uint8_t> data;
  MetadataMap metadata;

  PanoramaImageView view() const;
};

struct PanoramaSubscriptionStats {
  TransportMode transport_mode = TransportMode::SampleEndpoint;
  double observed_rate_hz = 0.0;
  double stitch_latency_ms = 0.0;
  double max_stitch_latency_ms = 0.0;
  std::uint64_t delivered_panoramas = 0;
  std::uint64_t dropped_frames = 0;
  std::uint64_t raw_parse_errors = 0;
  std::uint64_t decode_failures = 0;
  std::uint64_t stitch_failures = 0;
  std::uint64_t calibration_refresh_count = 0;
  std::string blocked_reason;
};

struct PanoramaFrameDelivery {
  OwnedPanoramaImage panorama;
  std::optional<OwnedFrame> source_frame;
  PanoramaSubscriptionStats stats;
};

using PanoramaFrameCallback = std::function<void(const PanoramaFrameDelivery&)>;

struct PanoramaSubscribeOptions {
  PanoramaStitchOptions stitch;
  double target_output_rate_hz = 10.0;
  std::uint32_t source_frame_stride = 2;
  std::size_t max_receive_queue_depth = 32;
  std::uint32_t first_frame_timeout_ms = 5000;
  bool require_streaming_transport = true;
  bool allow_sample_endpoint_fallback = false;
  bool include_source_frame = false;
};

Result<PanoramaCalibration> ParsePanoramaCalibrationJson(const std::string& calibration_json);
Result<PanoramaCalibration> MakePanoramaCalibration(const CalibrationStatus& calibration_status);
Result<OwnedPanoramaImage> GeneratePanorama(
    const std::vector<DecodedImageView>& images,
    const PanoramaCalibration& calibration,
    const PanoramaStitchOptions& options = {});
Result<OwnedPanoramaImage> GeneratePanorama(
    const std::vector<OwnedDecodedImage>& images,
    const PanoramaCalibration& calibration,
    const PanoramaStitchOptions& options = {});
const char* ToString(PanoramaBlendMode mode);
const char* ToString(PanoramaSeamMode mode);

}  // namespace orbvi_sdk
