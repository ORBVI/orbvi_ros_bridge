#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "orbvi_sdk/client.hpp"

namespace orbvi_sdk {

class CaptureSession;
class CaptureOutlet;
struct CapturedFrame;

struct McapRecordingOptions {
  std::string path;
  std::vector<std::string> camera_ids;
  // Recorder storage queues only; Start uses independent bounded source
  // queues. Configure custom source budgets with CaptureSession + Attach.
  std::size_t image_queue_capacity = 64;
  std::size_t imu_queue_capacity = 4096;
  std::size_t queue_bytes_limit = 128U * 1024U * 1024U;
  std::uint32_t startup_timeout_ms = 15000;
  std::uint32_t flush_interval_ms = 1000;
};

enum class McapRecordingMode {
  ImagesAndImu,
  ImagesOnly,
};

// FailOnLoss preserves admitted Host input and fails on known receive/write
// losses. Device-source completeness still needs independent boundary evidence.
enum class McapRecordingPolicy { BestEffort, FailOnLoss };

// Separate options preserve the layout of existing recording options/stats.
// Auto FailOnLoss saves IMU preroll before starting images, then waits for
// IMU time coverage after draining images. These are Host temporal checks,
// not proof of sensor-source completeness or a hard disk-IO stop deadline.
struct McapRecordingWindowOptions {
  std::uint32_t imu_preroll_ms = 100;
  std::uint32_t imu_tail_timeout_ms = 2000;
};

// Explicit MID360 point-cloud opt-in. These storage limits are independent of
// bounded source admission and share McapRecordingOptions::queue_bytes_limit.
// No LiDAR IMU is selected and no geometric calibration is required.
struct McapLidarRecordingOptions {
  bool enabled = false;
  std::size_t queue_capacity = 16;
  std::size_t queue_bytes_limit = 32U * 1024U * 1024U;
};

// Auto identifies UVC layouts and Client-provided sdk_device_model metadata;
// Start obtains it from the connected device. 4P never gains direction roles.
enum class McapRecordingProduct { Auto, FourV, FourVL, FourVLite, FourP };

struct McapRecordingStats {
  std::uint64_t received_frames = 0;
  std::uint64_t image_messages = 0;
  std::uint64_t imu_messages = 0;
  std::uint64_t dropped_frames = 0;
  std::uint64_t invalid_frames = 0;
  std::uint64_t sdk_receive_drops = 0;
  std::uint64_t reconnects = 0;
  std::uint64_t quality_events = 0;
  std::uint64_t bytes_written = 0;
  std::size_t queued_bytes = 0;
  std::size_t peak_queued_bytes = 0;
  bool active = false;
  bool ready = false;
  bool complete = false;
  std::string last_error;
  std::vector<std::string> camera_ids;
};

class McapRecorder {
public:
  McapRecorder();
  ~McapRecorder();
  McapRecorder(McapRecorder&&) noexcept;
  McapRecorder& operator=(McapRecorder&&) noexcept;
  McapRecorder(const McapRecorder&) = delete;
  McapRecorder& operator=(const McapRecorder&) = delete;

  // Existing manual Open overloads keep BestEffort behavior. The caller owns
  // source admission, receive policy and stop/drain when feeding record().
  static Result<McapRecorder> Open(
      const McapRecordingOptions& options, const MetadataMap& snapshots = {});
  static Result<McapRecorder> Open(
      const McapRecordingOptions& options, const MetadataMap& snapshots,
      McapRecordingMode mode);
  static Result<McapRecorder> Open(
      const McapRecordingOptions& options, const MetadataMap& snapshots,
      McapRecordingMode mode, McapRecordingProduct product);
  // RawFisheye (four physical cameras) or RectifiedFisheye (eight virtual
  // stereo views). Rectified recordings require the matching calibration
  // snapshot, including in ImagesOnly mode. Existing overloads select raw.
  static Result<McapRecorder> Open(
      const McapRecordingOptions& options, const MetadataMap& snapshots,
      McapRecordingMode mode, StreamId image_stream);
  static Result<McapRecorder> Open(
      const McapRecordingOptions& options, const MetadataMap& snapshots,
      McapRecordingMode mode, McapRecordingProduct product, StreamId image_stream);
  static Result<McapRecorder> Open(
      const McapRecordingOptions& options, const MetadataMap& snapshots,
      McapRecordingMode mode, McapRecordingProduct product, StreamId image_stream,
      McapRecordingPolicy policy);
  static Result<McapRecorder> Open(
      const McapRecordingOptions& options, const MetadataMap& snapshots,
      McapRecordingMode mode, McapRecordingProduct product, StreamId image_stream,
      McapRecordingPolicy policy, const McapLidarRecordingOptions& lidar);
  // Start overloads default to FailOnLoss: ordered SDK subscriptions, no
  // preview eviction, and failure on known receive/recording loss. A complete
  // file still has capture_integrity=UNKNOWN without source interval evidence.
  // FrameSync is also recorded for UvcVendorBulk and when explicitly advertised
  // by a current Network peer, including ImagesOnly. Legacy never adds it.
  // record() accepts current FrameSyncV2 frames; nextSdkFrame() restores them.
  // Completion metadata contains frame_sync_messages; received_frames includes
  // these frames while image_messages/imu_messages keep their existing meaning.
  static Result<McapRecorder> Start(Client& client,
                                    const McapRecordingOptions& options);
  static Result<McapRecorder> Start(Client& client,
                                    const McapRecordingOptions& options,
                                    McapRecordingMode mode);
  static Result<McapRecorder> Start(Client& client,
                                    const McapRecordingOptions& options,
                                    McapRecordingMode mode, StreamId image_stream);
  static Result<McapRecorder> Start(Client& client,
                                    const McapRecordingOptions& options,
                                    McapRecordingMode mode, StreamId image_stream,
                                    McapRecordingPolicy policy);
  static Result<McapRecorder> Start(Client& client,
                                    const McapRecordingOptions& options,
                                    McapRecordingMode mode, StreamId image_stream,
                                    McapRecordingPolicy policy,
                                    const McapRecordingWindowOptions& window);
  static Result<McapRecorder> Start(Client& client,
      const McapRecordingOptions& options, McapRecordingMode mode,
      StreamId image_stream, McapRecordingPolicy policy,
      const McapRecordingWindowOptions& window,
      const McapLidarRecordingOptions& lidar);
  // Attach to a prepared or running outlet as one atomic consumer group.
  // No historical preroll is fabricated; waitUntilReady establishes the window.
  // stop seals images, drains them, waits for IMU/FrameSync coverage, then drains
  // sidebands. The session retains source ownership throughout.
  static Result<McapRecorder> Attach(CaptureSession& session,
      const McapRecordingOptions& options,
      McapRecordingMode mode = McapRecordingMode::ImagesAndImu,
      StreamId image_stream = StreamId::RawFisheye);
  static Result<McapRecorder> Attach(CaptureSession& session,
      const McapRecordingOptions& options, McapRecordingMode mode,
      StreamId image_stream, const McapLidarRecordingOptions& lidar);
  Result<void> record(const FrameDelivery& delivery);
  static Result<McapRecorder> Attach(const CaptureOutlet& outlet,
      const McapRecordingOptions& options,
      McapRecordingMode mode = McapRecordingMode::ImagesAndImu,
      StreamId image_stream = StreamId::RawFisheye);
  static Result<McapRecorder> Attach(const CaptureOutlet& outlet,
      const McapRecordingOptions& options, McapRecordingMode mode,
      StreamId image_stream, const McapLidarRecordingOptions& lidar);
  Result<void> record(const CapturedFrame& delivery);
  Result<void> waitUntilReady();
  Result<McapRecordingStats> stop();
  McapRecordingStats stats() const;

private:
  struct Impl;
  explicit McapRecorder(std::shared_ptr<Impl> impl);
  std::shared_ptr<Impl> impl_;
};

}
