#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "orbvi_sdk/frame.hpp"
#include "orbvi_sdk/lidar_calibration.hpp"
#include "orbvi_sdk/livox.hpp"
#include "orbvi_sdk/image.hpp"
#include "orbvi_sdk/depth.hpp"
#include "orbvi_sdk/panorama.hpp"
#include "orbvi_sdk/transport_options.hpp"
#include "orbvi_sdk/uvc_contract.hpp"

namespace orbvi_sdk {

class BulkReader;
class UvcReader;

namespace detail {
class BulkOtaRuntime;
class ClientTestPeer;
}

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
  // Dual-transport selection (Chapter 1). Defaults to Network for both, which
  // is fully backward-compatible with the existing network-only client. UVC
  // is an explicit Linux V4L2 transport; protocol Bulk and Bulk OTA use
  // distinct exact USB interfaces.
  DeviceOptions device_options;
  // Selects one USB composite parent. Child UVC functions are resolved by
  // iInterface. UvcVendorBulk requires paired video+metadata nodes; UvcOnly
  // binds the single video node without inventing FrameSync metadata.
  // Protocol Bulk requires exact nonzero VID/PID and a nonempty iSerial. The
  // SDK never falls back to Network when this identity is absent or rejected.
  UsbCompositeSelector usb_selector;
  // Backward-compatible selector hint. If set, this must identify the USB
  // composite sysfs parent. A single /dev/videoN child is ambiguous and is
  // rejected before any control-plane probe.
  std::string usb_device_path;
  // UvcOnly has no control identity: explicitly select the physical layout.
  // UvcVendorBulk discovers 4P from device info instead.
  std::string uvc_only_raw_layout = kRawAtlasLayout;
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
  std::string release_profile;
  std::string product;
  std::string device_model;
  std::string hardware_profile;
  std::string app_version;
  std::string protocol_version;
  std::vector<std::string> supported_streams;
  std::vector<std::string> unsupported_streams;
  std::string raw_json;
};

struct StreamStatus {
  StreamId stream_id = StreamId::Unknown;
  FrameFormat format = FrameFormat::Unknown;
  std::string status;
  std::string activity_state;
  std::string source_status;
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

struct DataPlaneStatus {
  std::string activity_state;
  bool running = false;
  std::uint64_t active_clients = 0;
  std::vector<StreamId> active_streams;
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
  DataPlaneStatus data_plane;
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
  // Returns board-declared LiDAR IMU units, separate from factory calibration.
  Result<LidarImuParameters> getLidarImuParameters();
  // Fetches the device calibration bundle and exposes the factory
  // LiDAR/camera transforms as typed data instead of requiring JSON parsing.
  Result<LidarCalibration> getLidarCalibration(const std::string& version = "");
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
  Result<OwnedPanoramaImage> getPanorama(
      PanoramaSourceMode source_mode,
      const PanoramaStitchOptions& options = {});
  Result<PanoramaSubscriptionHandle> subscribePanorama(PanoramaFrameCallback callback);
  Result<PanoramaSubscriptionHandle> subscribePanorama(
      PanoramaSubscribeOptions options,
      PanoramaFrameCallback callback);
  Result<PanoramaSubscriptionHandle> subscribePanorama(
      PanoramaSourceMode source_mode,
      PanoramaSubscribeOptions options,
      PanoramaFrameCallback callback);
  Result<PanoramaSubscriptionHandle> subscribePanorama(
      PanoramaSubscribeOptions options,
      PanoramaDepthProvider depth_provider,
      PanoramaFrameCallback callback);
  Result<PanoramaSubscriptionHandle> subscribePanorama(
      PanoramaSourceMode source_mode,
      PanoramaSubscribeOptions options,
      PanoramaDepthProvider depth_provider,
      PanoramaFrameCallback callback);

 private:
  friend class detail::ClientTestPeer;

  // Private production-path test seam. Tests inject a BulkReader carrying a
  // fake UsbBackend; normal callers always use the public constructor and the
  // default libusb backend.
  Client(
      ClientOptions options,
      std::function<void(BulkReader*)> prepare_bulk_reader_for_open,
      std::function<void(detail::BulkOtaRuntime*)>
          prepare_bulk_ota_for_open = {});
  // Private production-path test seam for a fake V4L2 backend. Normal callers
  // always use the public constructor and dynamic composite discovery.
  Client(
      ClientOptions options,
      std::function<void(UvcReader*)> prepare_uvc_reader_for_open);
  // Private pure-USB production-path test seam. It lets one test exercise the
  // real UVC + Vendor Bulk profile while injecting all three hardware-facing
  // backends; normal callers cannot construct mixed transport states.
  Client(
      ClientOptions options,
      std::function<void(BulkReader*)> prepare_bulk_reader_for_open,
      std::function<void(detail::BulkOtaRuntime*)>
          prepare_bulk_ota_for_open,
      std::function<void(UvcReader*)> prepare_uvc_reader_for_open);

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace orbvi_sdk
