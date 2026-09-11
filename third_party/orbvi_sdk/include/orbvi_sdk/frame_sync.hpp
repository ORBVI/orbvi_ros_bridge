#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace orbvi_sdk {

// Host-side mirror of the device FrameSyncV2 wire contract. The two codebases
// do not share headers, so the on-wire layout MUST stay byte-for-byte identical
// with src/orbvi_sdk_core. Equality is asserted by cross-side golden-fixture
// tests in both packages.
//
// Design principle: capture_seq is the authoritative frame identity. UVC PTS
// is only a transport-layer time index used to correlate a V4L2 UVC payload
// header with the capture cycle. The host matches on
// (boot_id, uvc_stream_index, pts_ticks) -> capture_seq, then uses
// (boot_id, capture_seq) for all business-layer synchronization.

constexpr std::uint32_t kFrameSyncMagic = 0x4f465359u;
constexpr std::uint16_t kFrameSyncProtocolVersion = 2;

constexpr std::size_t kFrameSyncMaxCameras = 6;
constexpr std::size_t kFrameSyncStreamCount = 7;
constexpr std::size_t kFrameSyncUvcCount = 4;

// Legacy profiles use a 90 kHz, 32-bit UVC counter. X5 4P BSP uses
// 48 MHz with microsecond truncation; its FrameSync carries that exact counter.
// PTS matching is opaque equality; product metadata defines the clock. The device generates
// pts_ticks via PtsTicksFromNs(trigger_monotonic_ns - boot_monotonic_epoch_ns),
// using the CLOCK_MONOTONIC domain (NOT capture_timestamp_ns which is
// Realtime/PTP — PTP corrections would make UVC PTS jump/regress). boot epoch
// is unique per power-on, NOT reset by USB stop/start. pts_ticks == 0 is legal;
// validity is determined solely by uvc_valid_mask.
constexpr std::uint32_t kUvcPtsClockHz = 90'000;
constexpr std::uint8_t kUvcPtsCounterBits = 32;

// Overflow-safe conversion from device nanoseconds to UVC PTS ticks. A naive
// delta_ns * 90000 / 1e9 in uint64_t overflows after ~56.93h. Uses __uint128_t
// (or a seconds+fraction split fallback) to stay correct for the full uint64_t
// range before truncating to the 32-bit counter.
inline std::uint32_t PtsTicksFromNs(std::uint64_t delta_ns) {
#if defined(__SIZEOF_INT128__)
  __uint128_t product =
      static_cast<__uint128_t>(delta_ns) * static_cast<__uint128_t>(kUvcPtsClockHz);
  const std::uint64_t ticks =
      static_cast<std::uint64_t>(product / static_cast<__uint128_t>(1'000'000'000ULL));
  return static_cast<std::uint32_t>(ticks);
#else
  const std::uint64_t whole_seconds = delta_ns / 1'000'000'000ULL;
  const std::uint64_t frac_ns = delta_ns % 1'000'000'000ULL;
  const std::uint64_t ticks =
      whole_seconds * kUvcPtsClockHz + (frac_ns * kUvcPtsClockHz) / 1'000'000'000ULL;
  return static_cast<std::uint32_t>(ticks);
#endif
}

enum class FrameSyncStreamIndex : std::uint8_t {
  RawFisheye = 0,
  RectifiedFisheye = 1,
  Panorama = 2,
  Bev = 3,
  Disparity = 4,
  VioPose = 5,
  Imu = 6,
};

// Logical UVC video stream index — stable function ID, NOT a /dev/videoN node.
// Host resolves the actual device via /dev/v4l/by-id or by-path; PTS is read
// from the V4L2_META_FMT_UVC metadata node, not v4l2_buffer.timestamp.
// All four slots are frozen. Whether a slot is advertised is capability/profile
// driven; format changes add a profile and never renumber the function ID.
enum class FrameSyncUvcIndex : std::uint8_t {
  RawFisheye = 0,
  RectifiedFisheye = 1,
  Panorama = 2,
  Bev = 3,
};

inline std::uint32_t FrameSyncStreamBit(FrameSyncStreamIndex index) {
  return 1u << static_cast<unsigned>(index);
}

inline bool FrameSyncStreamValid(std::uint32_t stream_valid_mask,
                                 FrameSyncStreamIndex index) {
  return (stream_valid_mask & FrameSyncStreamBit(index)) != 0u;
}

inline bool FrameSyncCameraValid(std::uint32_t camera_valid_mask,
                                 std::size_t camera_id) {
  if (camera_id >= kFrameSyncMaxCameras) return false;
  return (camera_valid_mask & (1u << static_cast<unsigned>(camera_id))) != 0u;
}

inline std::uint32_t FrameSyncUvcBit(FrameSyncUvcIndex index) {
  return 1u << static_cast<unsigned>(index);
}

inline bool FrameSyncUvcValid(std::uint32_t uvc_valid_mask,
                              FrameSyncUvcIndex index) {
  return (uvc_valid_mask & FrameSyncUvcBit(index)) != 0u;
}

// Signed modular delta for 32-bit PTS wrap-around comparison. A delta of 0
// means an exact match. The host keeps a small bounded match window (e.g. the
// most recent 64 frames per UVC stream) and flushes it on boot_id change.
inline std::int32_t PtsDelta(std::uint32_t lhs, std::uint32_t rhs) {
  return static_cast<std::int32_t>(lhs - rhs);
}

struct UvcFrameRef {
  // Raw UVC payload header PTS ticks (90 kHz counter). 0 is legal.
  std::uint32_t pts_ticks = 0;
  std::uint32_t reserved = 0;
  // UVC emission counter since boot (resets with boot_id). TRANSPORT DIAGNOSTIC
  // only — must NOT be used for cross-transport correlation (use capture_seq).
  std::uint64_t uvc_emit_seq = 0;
};

struct FrameSyncV2 {
  std::uint32_t magic = kFrameSyncMagic;
  std::uint16_t version = kFrameSyncProtocolVersion;
  std::uint16_t reserved16 = 0;
  std::uint64_t boot_id = 0;
  // Capture cycle counter = Camera Graph sync_seq = CAMERA_TRIGGER.seq.
  // The authoritative cross-transport frame identity.
  std::uint64_t capture_seq = 0;
  // Realtime/PTP domain (from CAMERA_TRIGGER header.stamp). Business sync.
  std::uint64_t capture_timestamp_ns = 0;
  // CLOCK_MONOTONIC domain (from trigger_monotonic_ns). UVC PTS input only.
  std::uint64_t trigger_monotonic_ns = 0;
  std::uint64_t trigger_seq = 0;
  std::uint8_t trigger_seq_valid = 0;
  std::uint8_t reserved8[3] = {0, 0, 0};
  // Bit i == camera_id i (0..5). Fisheye cameras are id 3/2/4/5 (front/right/
  // rear/left).
  std::uint32_t camera_valid_mask = 0;
  // Indexed directly by camera_id 0..5.
  std::int64_t per_camera_offset_ns[kFrameSyncMaxCameras] = {0, 0, 0, 0, 0, 0};
  std::uint64_t stream_seq[kFrameSyncStreamCount] = {0, 0, 0, 0, 0, 0, 0};
  std::uint32_t stream_valid_mask = 0;
  std::uint32_t calibration_hash_tag = 0;
  std::uint32_t sync_flags = 0;
  UvcFrameRef uvc[kFrameSyncUvcCount] = {};
  std::uint32_t uvc_valid_mask = 0;
  std::uint32_t reserved_tail = 0;
  std::uint32_t crc32 = 0;
};

enum class FrameSyncDecodeStatus {
  Ok,
  NeedMoreData,
  InvalidMagic,
  UnsupportedVersion,
  InvalidSize,
  CrcMismatch,
};

const char* ToString(FrameSyncDecodeStatus status);

struct FrameSyncDecodeResult {
  FrameSyncDecodeStatus status = FrameSyncDecodeStatus::NeedMoreData;
  FrameSyncV2 frame_sync;
  std::size_t bytes_consumed = 0;
  std::string error;

  bool ok() const { return status == FrameSyncDecodeStatus::Ok; }
};

std::vector<std::uint8_t> EncodeFrameSync(const FrameSyncV2& frame_sync);
FrameSyncDecodeResult DecodeFrameSync(const std::uint8_t* data,
                                      std::size_t size);
FrameSyncDecodeResult DecodeFrameSync(const std::vector<std::uint8_t>& data);

// Field-level sanity check mirroring the device-side ValidateFrameSync: rejects
// bad magic/version, zero boot_id/capture_seq/capture_timestamp_ns, empty
// camera_valid_mask, and claimed-valid streams with zero sequence. Does NOT
// reject pts_ticks == 0 (legal value).
bool ValidateFrameSync(const FrameSyncV2& frame_sync,
                       std::string* reason = nullptr);

}  // namespace orbvi_sdk
