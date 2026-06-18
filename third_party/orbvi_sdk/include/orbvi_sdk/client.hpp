#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "orbvi_sdk/frame.hpp"
#include "orbvi_sdk/image.hpp"
#include "orbvi_sdk/depth.hpp"
#include "orbvi_sdk/panorama.hpp"

namespace orbvi_sdk {

struct ClientOptions {
  std::string host = "127.0.0.1";
  std::uint16_t control_port = 18088;
  std::uint32_t connect_timeout_ms = 2000;
  std::uint32_t read_timeout_ms = 2000;
  std::size_t max_receive_queue_depth = 32;
  DecodePolicy default_decode_policy;
  bool allow_sample_endpoint_fallback = false;
  std::uint32_t ota_preferred_chunk_bytes = 4u * 1024u * 1024u;
  bool ota_low_memory_upload = false;
  std::uint32_t ota_finalize_timeout_ms = 180000;
};

struct SubscribeOptions {
  DecodePolicy decode_policy;
  std::size_t max_receive_queue_depth = 32;
  std::uint32_t first_frame_timeout_ms = 5000;
  bool require_streaming_transport = false;
  bool allow_sample_endpoint_fallback = false;
};

struct DeviceInfo {
  std::string device_name;
  std::string app_version;
  std::string protocol_version;
  std::vector<std::string> supported_streams;
  std::vector<std::string> unsupported_streams;
};

struct StreamStatus {
  StreamId stream_id = StreamId::Unknown;
  FrameFormat format = FrameFormat::Unknown;
  std::string status;
  std::string real_source_status;
  std::string blocked_reason;
  double target_rate_hz = 0.0;
  double observed_rate_hz = 0.0;
  std::uint64_t payload_size_bytes = 0;
  std::size_t queue_depth = 0;
  std::uint64_t dropped_frames = 0;
  std::uint64_t produced_frames = 0;
  std::uint64_t rejected_frames = 0;
  double send_latency_ms = 0.0;
  std::string note;

  std::vector<int> camera_order;
  std::uint32_t image_count = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t jpeg_quality = 0;
  std::string calibration_version;
  std::string source_pixel_format;
  std::string jpeg_source;
  bool software_jpeg_encode = false;
  double timestamp_span_ms = 0.0;
  std::string jpeg_encoder_status;
  bool native_raw_producer_running = false;
  std::string native_raw_producer_diagnostic;

  std::string disparity_layout;
  std::uint32_t disparity_tile_count = 0;
  std::uint32_t disparity_tile_width = 0;
  std::uint32_t disparity_tile_height = 0;
  double disparity_scale = 0.0;
  std::string invalid_value;
};

struct DeviceCalibrationStatus {
  std::string status;
  std::string calibration_version;
  std::string calibration_hash;
  std::string calibration_hash_status;
  std::string source;
};

struct DeviceProcessStatus {
  bool available = false;
  std::uint64_t fd_count = 0;
  std::uint64_t thread_count = 0;
  std::uint64_t rss_kb = 0;
};

struct DeferredCapabilityStatus {
  std::string name;
  std::string status;
  std::string blocked_reason;
  std::string note;
};

struct DeviceStatus {
  std::string state;
  std::uint64_t active_sessions = 0;
  DeviceCalibrationStatus calibration;
  DeviceProcessStatus process;
  std::vector<DeferredCapabilityStatus> deferred_capabilities;
  std::vector<StreamStatus> streams;
  std::string raw_json;
};

struct TimeStatus {
  bool synced = false;
  std::string time_source;
  std::uint64_t last_sync_age_ms = 0;
  std::string status;
};

struct CalibrationStatus {
  std::string status;
  std::string calibration_version;
  std::string calibration_hash;
  std::string source;
  std::uint64_t last_fetch_timestamp_ns = 0;
  std::string raw_json;
};

struct OtaDiagnosticCounters {
  std::string last_failure_stage;
  int last_error_code = 0;
  bool payload_cleanup_success = true;
};

struct OtaProgress {
  std::string session_id;
  std::string state;
  double progress = 0.0;
  std::string component;
  std::string message;
  std::uint64_t uploaded_bytes = 0;
  std::uint64_t total_bytes = 0;
  int error_code = 0;
  bool can_cancel = false;
  OtaDiagnosticCounters diagnostic_counters;
  std::string raw_json;
};

struct OtaCommandResult {
  bool ok = false;
  std::string session_id;
  std::string state;
  std::string message;
  std::uint32_t chunk_size = 0;
  std::string upload_transport;
  std::string upload_host;
  std::uint16_t upload_port = 0;
  std::uint64_t resume_offset = 0;
  std::uint64_t uploaded_bytes = 0;
  std::uint64_t total_bytes = 0;
  int error_code = 0;
  bool can_cancel = false;
  std::string raw_json;
};

struct FrameDelivery {
  OwnedFrame frame;
  std::optional<OwnedDecodedImage> decoded_image;
  std::vector<OwnedDecodedImage> decoded_images;
  SubscriptionStats stats;
};

using FrameCallback = std::function<void(const FrameDelivery&)>;

struct SubscriptionState;
struct DepthSubscriptionState;
struct PanoramaSubscriptionState;

class SubscriptionHandle {
 public:
  SubscriptionHandle() = default;
  explicit SubscriptionHandle(std::shared_ptr<SubscriptionState> state);
  SubscriptionHandle(const SubscriptionHandle&) = delete;
  SubscriptionHandle& operator=(const SubscriptionHandle&) = delete;
  SubscriptionHandle(SubscriptionHandle&& other) noexcept;
  SubscriptionHandle& operator=(SubscriptionHandle&& other) noexcept;
  ~SubscriptionHandle();

  void stop();
  bool active() const;
  SubscriptionStats stats() const;

 private:
  std::shared_ptr<SubscriptionState> state_;
};

class DepthSubscriptionHandle {
 public:
  DepthSubscriptionHandle() = default;
  explicit DepthSubscriptionHandle(std::shared_ptr<DepthSubscriptionState> state);
  DepthSubscriptionHandle(const DepthSubscriptionHandle&) = delete;
  DepthSubscriptionHandle& operator=(const DepthSubscriptionHandle&) = delete;
  DepthSubscriptionHandle(DepthSubscriptionHandle&& other) noexcept;
  DepthSubscriptionHandle& operator=(DepthSubscriptionHandle&& other) noexcept;
  ~DepthSubscriptionHandle();

  void stop();
  bool active() const;
  DepthSubscriptionStats stats() const;

 private:
  std::shared_ptr<DepthSubscriptionState> state_;
};

class PanoramaSubscriptionHandle {
 public:
  PanoramaSubscriptionHandle() = default;
  explicit PanoramaSubscriptionHandle(std::shared_ptr<PanoramaSubscriptionState> state);
  PanoramaSubscriptionHandle(const PanoramaSubscriptionHandle&) = delete;
  PanoramaSubscriptionHandle& operator=(const PanoramaSubscriptionHandle&) = delete;
  PanoramaSubscriptionHandle(PanoramaSubscriptionHandle&& other) noexcept;
  PanoramaSubscriptionHandle& operator=(PanoramaSubscriptionHandle&& other) noexcept;
  ~PanoramaSubscriptionHandle();

  void stop();
  bool active() const;
  PanoramaSubscriptionStats stats() const;

 private:
  std::shared_ptr<PanoramaSubscriptionState> state_;
};

class Client {
 public:
  explicit Client(ClientOptions options);
  ~Client();

  Client(const Client&) = delete;
  Client& operator=(const Client&) = delete;
  Client(Client&&) noexcept;
  Client& operator=(Client&&) noexcept;

  Result<void> connect();
  void disconnect();
  bool connected() const;

  Result<DeviceInfo> getDeviceInfo();
  Result<DeviceStatus> getStatus();
  Result<TimeStatus> getTimeStatus();
  Result<CalibrationStatus> getCalibration(const std::string& version = "");
  Result<OtaProgress> getOtaProgress();
  Result<std::string> getOtaLog();
  Result<OtaCommandResult> beginOta(const std::string& package_path);
  Result<OtaCommandResult> uploadOtaPackage(const std::string& package_path);
  Result<OtaCommandResult> cancelOta();
  Result<OtaCommandResult> commitOta();

  Result<SubscriptionHandle> subscribe(StreamId stream, FrameCallback callback);
  Result<SubscriptionHandle> subscribe(
      StreamId stream,
      SubscribeOptions options,
      FrameCallback callback);
  Result<DepthMap> getDepthMap(const DepthGenerationOptions& options = {});
  Result<DepthSubscriptionHandle> subscribeDepthMap(DepthFrameCallback callback);
  Result<DepthSubscriptionHandle> subscribeDepthMap(
      DepthSubscribeOptions options,
      DepthFrameCallback callback);
  Result<OwnedPanoramaImage> getPanorama(const PanoramaStitchOptions& options = {});
  Result<PanoramaSubscriptionHandle> subscribePanorama(PanoramaFrameCallback callback);
  Result<PanoramaSubscriptionHandle> subscribePanorama(
      PanoramaSubscribeOptions options,
      PanoramaFrameCallback callback);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace orbvi_sdk
