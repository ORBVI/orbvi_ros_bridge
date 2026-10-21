#pragma once

#include <string>
#include <vector>
#include "orbvi_sdk/frame.hpp"

namespace orbvi_sdk {

// Provisioning result; device_sn is the saved identity (or active fallback).
// Reboot the device when reboot_required is true to apply it to all transports.
struct DeviceSnStatus {
  std::string device_sn;
  std::string active_device_sn;
  bool reboot_required = false;
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
  std::string sdk_contract;
  // Saved SN, visible immediately after provisioning. The running identity and
  // reboot requirement are available through getDeviceSn() (also raw_json).
  std::string device_sn;
  // Deprecated source compatibility field. Prefer hardware_profile.
  std::string camera_profile;
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

struct AlgorithmStatus {
  // Changes after every completed board algorithm reset or process restart.
  std::uint64_t generation = 0;
  std::uint64_t timestamp_ns = 0;  // Device CLOCK_REALTIME, also used by cloud frames.
  // Empty on legacy firmware. Describes the board world-coordinate convention,
  // not the ROS frame name or whether an individual sample is initialized.
  std::string world_frame;
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


}  // namespace orbvi_sdk
