#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

#include "orbvi_sdk/frame_sync.hpp"

namespace orbvi_sdk {

// Host-side UVC PTS → capture_seq matcher (Chapter 6).
//
// The host receives FrameSync records over Network/Bulk and UVC frames over
// V4L2. Each UVC frame carries a PTS in its payload header (read from
// V4L2_META_FMT_UVC, NOT v4l2_buffer.timestamp). The matcher correlates a UVC
// PTS with the most recent FrameSync records to recover capture_seq.
//
// Match key: (boot_id, uvc_stream_index, pts_ticks) → capture_seq
//
// The matcher keeps a bounded sliding window of recent FrameSync records (one
// per capture cycle). When a UVC frame arrives, it scans the window for an
// exact PTS match (PtsDelta == 0). If boot_id changes (device reboot), the
// window is flushed immediately.

constexpr std::size_t kPtsMatcherWindowSize = 64;

class PtsMatcher {
 public:
  PtsMatcher() = default;

  // Record a FrameSync received from Network/Bulk. If boot_id differs from the
  // current window, flush first.
  void OnFrameSync(const FrameSyncV2& fs);

  // Try to match a UVC frame's PTS to a capture_seq. Returns the capture_seq if
  // an exact match is found in the window, or std::nullopt if no match.
  std::optional<std::uint64_t> Match(std::uint32_t pts_ticks,
                                     FrameSyncUvcIndex uvc_index) const;

  // Number of FrameSync records currently in the window.
  std::size_t window_size() const { return window_.size(); }

  // Current boot_id (0 if none observed yet).
  std::uint64_t boot_id() const { return boot_id_; }

  // Clear the window (e.g. on USB re-enumeration or explicit reset).
  void Clear();

 private:
  struct Entry {
    std::uint64_t capture_seq = 0;
    // PTS for each UVC stream (only meaningful if that stream was valid).
    std::array<std::uint32_t, kFrameSyncUvcCount> pts = {};
    std::uint32_t uvc_valid_mask = 0;
  };

  std::uint64_t boot_id_ = 0;
  std::deque<Entry> window_;
  // Mutex protects boot_id_ and window_ — OnFrameSync (Network/Bulk thread)
  // and Match (UVC dequeue thread) run concurrently.
  mutable std::mutex mutex_;
};

inline void PtsMatcher::OnFrameSync(const FrameSyncV2& fs) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fs.boot_id != boot_id_) {
    boot_id_ = fs.boot_id;
    window_.clear();
  }

  Entry e;
  e.capture_seq = fs.capture_seq;
  e.uvc_valid_mask = fs.uvc_valid_mask;
  for (std::size_t i = 0; i < kFrameSyncUvcCount; ++i) {
    e.pts[i] = fs.uvc[i].pts_ticks;
  }
  for (auto& existing : window_) {
    if (existing.capture_seq == e.capture_seq) {
      for (std::size_t index = 0; index < kFrameSyncUvcCount; ++index)
        if (e.uvc_valid_mask & (1U << index)) existing.pts[index] = e.pts[index];
      existing.uvc_valid_mask |= e.uvc_valid_mask;
      return;
    }
  }
  window_.push_back(std::move(e));
  while (window_.size() > kPtsMatcherWindowSize) {
    window_.pop_front();
  }
}

inline std::optional<std::uint64_t> PtsMatcher::Match(
    std::uint32_t pts_ticks, FrameSyncUvcIndex uvc_index) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto idx = static_cast<std::size_t>(uvc_index);
  // Scan from most recent backwards for the best chance of a hit.
  for (auto it = window_.rbegin(); it != window_.rend(); ++it) {
    if (!FrameSyncUvcValid(it->uvc_valid_mask, uvc_index)) {
      continue;
    }
    if (PtsDelta(pts_ticks, it->pts[idx]) == 0) {
      return it->capture_seq;
    }
  }
  return std::nullopt;
}

inline void PtsMatcher::Clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  window_.clear();
  boot_id_ = 0;
}

}  // namespace orbvi_sdk
