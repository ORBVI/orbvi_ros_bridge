#pragma once
#include "orbvi_sdk/capture_types.hpp"
#include "orbvi_sdk/imu.hpp"
#include "orbvi_sdk/image.hpp"

namespace orbvi_sdk {
struct CaptureClockIdentity {
  PeerContract peer = PeerContract::Current;
  TimestampDomain domain = TimestampDomain::Unknown;
  std::string boot_id;
  std::uint64_t session_epoch = 0;
  bool operator==(const CaptureClockIdentity& other) const;
};
Result<CaptureClockIdentity> ResolveCaptureClock(const FrameView& frame, std::uint64_t session_epoch);
struct ImageImuWindowOptions {
  std::size_t max_imu_samples = 512;
  std::size_t max_pending_images = 8;
  std::size_t max_pending_image_bytes = 64U * 1024U * 1024U;
  std::size_t max_output_imu_samples = 4096; // Aggregate per push, across returned windows.
  std::uint64_t max_imu_gap_ns = 20000000ULL;
  bool allow_initial_prefix_skip = true;
  // Ignore gaps belonging to other consumers. Pass IDs of this window's inputs.
  std::vector<std::uint64_t> consumer_ids;
};
// Temporal coverage only: no interpolation, frame synchronization association,
// or hardware trigger proof is created. IMU values retain their source units.
struct ImageImuWindow {
  CapturedFrame image;
  ImageTimeRange exposure;
  CaptureClockIdentity clock;
  std::vector<ImuSample> imu; // Includes one sample on each side, or exact endpoints.
};
struct ImageImuWindowReport {
  std::uint64_t completed = 0, skipped_prefix = 0;
  std::size_t pending_images = 0, retained_imu = 0, pending_image_bytes = 0;
  std::string error;
};
class ImageImuWindowAssembler {
 public:
  explicit ImageImuWindowAssembler(ImageImuWindowOptions options = {});
  ~ImageImuWindowAssembler();
  ImageImuWindowAssembler(const ImageImuWindowAssembler&) = delete;
  ImageImuWindowAssembler& operator=(const ImageImuWindowAssembler&) = delete;
  Result<std::vector<ImageImuWindow>> push(const CapturedFrame& frame);
  // Gap/source failure invalidates the window; a new epoch needs a new assembler.
  Result<void> observe(const CaptureEvent& event);
  Result<void> finish();
  ImageImuWindowReport report() const;
 private:
  friend class ImageImuIntervalAssembler;
  ImageImuWindowAssembler(ImageImuWindowOptions options, bool consecutive_intervals);
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
// Consecutive image-end intervals [previous.last_ns, current.last_ns].
// First image anchors the interval; no pre-first-image interval is invented.
struct ImageImuInterval {
  CapturedFrame image;
  ImageTimeRange interval;
  CaptureClockIdentity clock;
  std::vector<ImuSample> imu; // Inclusive bracketing samples; adjacent endpoints may repeat.
};
class ImageImuIntervalAssembler {
 public:
  explicit ImageImuIntervalAssembler(ImageImuWindowOptions options = {});
  ~ImageImuIntervalAssembler();
  Result<std::vector<ImageImuInterval>> push(const CapturedFrame& frame);
  Result<void> observe(const CaptureEvent& event);
  Result<void> finish();
  ImageImuWindowReport report() const;
 private:
  std::unique_ptr<ImageImuWindowAssembler> impl_;
};
}  // namespace orbvi_sdk
