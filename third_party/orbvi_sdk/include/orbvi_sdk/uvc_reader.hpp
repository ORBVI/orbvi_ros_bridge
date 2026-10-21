#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "orbvi_sdk/frame_sync.hpp"
#include "orbvi_sdk/pts_matcher.hpp"
#include "orbvi_sdk/status.hpp"
#include "orbvi_sdk/uvc_contract.hpp"

namespace orbvi_sdk {

namespace detail {
class UvcBackend;
class UvcReaderTestPeer;
class SubscriptionTransportAccess;
struct UvcReaderRunState;
}  // namespace detail

inline constexpr std::uint32_t kUvcReaderMaxDequeueTimeoutMs = 10000;

struct UvcReaderConfig {
  UsbCompositeSelector usb_selector;
  FrameSyncUvcIndex function_id = FrameSyncUvcIndex::RawFisheye;
  std::uint16_t format_profile_id = 0;
  // Optional pre-resolved nodes for an embedding that already performed the
  // exact composite/iInterface discovery. Normal callers leave these empty so
  // Open() resolves the current dynamic /dev/videoN pair every time.
  std::string video_device;
  std::string metadata_device;
  // Full UVC + Vendor Bulk joins V4L2 metadata PTS to FrameSync. The X5
  // single-function UVC-only profile has no metadata node and delivers the
  // video buffer timestamp explicitly as unsynchronized Host time.
  bool require_metadata = true;
  std::uint32_t buffer_count = 4;
  std::uint32_t dequeue_timeout_ms = 200;
  std::uint32_t max_pending_sequences = 8;
  std::uint32_t framesync_wait_ms = 50;
};

struct UvcFrame {
  std::vector<std::uint8_t> payload;
  std::uint32_t pts_ticks = 0;
  bool pts_valid = false;
  std::uint64_t capture_seq = 0;
  bool synced = false;
  std::uint64_t host_timestamp_ns = 0;
  TimestampDomain host_timestamp_domain = TimestampDomain::Unknown;
  // Device/session epoch captured at the video+metadata join boundary. Client
  // delivery gates use both values so a queued frame from an earlier boot can
  // never match a repeated PTS/capture_seq in the next boot.
  std::uint64_t boot_id = 0;
  std::uint64_t generation = 0;
};

struct UvcReaderStats {
  std::uint64_t captured_frames = 0;
  std::uint64_t delivered_frames = 0;
  std::uint64_t dropped_frames = 0;
  std::uint64_t metadata_errors = 0;
  std::uint64_t dequeue_errors = 0;
  std::uint64_t v4l2_error_frames = 0;
  std::size_t pending_video_sequences = 0;
  std::size_t pending_metadata_sequences = 0;
  std::size_t delivery_queue_depth = 0;
  std::size_t max_delivery_queue_depth = 0;
};

enum class UvcReaderStatus {
  Ok,
  DeviceNotFound,
  OpenFailed,
  FormatNotSupported,
  StreamOnFailed,
  MetadataOpenFailed,
  InvalidSelector,
  AmbiguousDevice,
  PermissionDenied,
  DescriptorMismatch,
  DeviceIdentityMismatch,
  CloseFailed,
  RecordingIncomplete,
};

const char* ToString(UvcReaderStatus status);

using UvcFrameCallback = std::function<void(const UvcFrame&)>;

struct UvcStreamEpoch {
  std::uint64_t boot_id = 0;
  std::uint64_t generation = 0;
};

using UvcEpochProvider = std::function<UvcStreamEpoch()>;
using UvcFailureCallback =
    std::function<void(UvcReaderStatus, const UvcStreamEpoch&)>;

// Host UVC reader with proper thread lifecycle. Start() spawns a real
// std::thread that dequeues frames; Stop() signals and joins it.
class UvcReader {
 public:
  UvcReader();
  ~UvcReader();

  UvcReader(const UvcReader&) = delete;
  UvcReader& operator=(const UvcReader&) = delete;

  UvcReaderStatus Open(const UvcReaderConfig& config);
  bool Start(UvcFrameCallback callback, const PtsMatcher* pts_matcher,
             std::uint64_t boot_id);
  bool Start(UvcFrameCallback callback, const PtsMatcher* pts_matcher,
             std::uint64_t boot_id, UvcEpochProvider epoch_provider,
             UvcFailureCallback failure_callback);
  void Stop();
  bool IsRunning() const;
  const UvcReaderConfig& config() const { return config_; }
  UvcReaderStats stats() const;
  UvcReaderStatus last_status() const;

 private:
  friend class detail::UvcReaderTestPeer;
  friend class detail::SubscriptionTransportAccess;

  UvcReaderConfig config_;
  std::unique_ptr<detail::UvcBackend> injected_backend_;
  std::shared_ptr<detail::UvcReaderRunState> state_;
  std::thread capture_thread_;
  std::thread delivery_thread_;
  mutable std::mutex lifecycle_mutex_;
  std::condition_variable lifecycle_cv_;
  bool stopping_ = false;
};

}  // namespace orbvi_sdk
