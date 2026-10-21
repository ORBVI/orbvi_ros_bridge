#pragma once
#include "orbvi_sdk/capture.hpp"

namespace orbvi_sdk {
using SharedCompressedImages =
    std::vector<std::shared_ptr<const CompressedImage>>;
struct CaptureImageSharingReport {
  std::uint64_t requests = 0, hits = 0, transformed_tiles = 0;
  std::size_t indexed_frames = 0;
};
// A retained frame may contain four tiles, even when its caller requested one.
// This conservative charge bounds output JPEGs and copied identity strings.
constexpr std::size_t kCaptureImageFrameBudget = 40U * 1024U * 1024U;
struct CaptureImageConsumerOptions {
  std::size_t max_retained_frames = 2;
  std::size_t max_concurrent_transforms = 1;
};
struct CaptureImagePoolOptions {
  std::size_t max_indexed_frames = 32;
  std::size_t max_reserved_frames = 16;
  std::size_t max_concurrent_transforms = 4;
  CaptureImageConsumerOptions default_consumer;
};
struct CaptureImageResourceReport {
  std::uint64_t requests = 0, hits = 0, transformed_tiles = 0;
  std::size_t live_bytes = 0, peak_bytes = 0;
  std::size_t reserved_bytes = 0, capacity_bytes = 0;
  std::size_t active_transforms = 0, peak_transforms = 0;
  std::uint64_t budget_rejections = 0, transform_ns = 0, max_transform_ns = 0;
};
struct CaptureImageConsumerReport {
  std::size_t held_frames = 0, active_transforms = 0;
  std::uint64_t budget_rejections = 0;
};
namespace detail {
struct CaptureImageQuota;
}
// An independent reservation within a shared weak cache. Keeping JPEGs pins
// this consumer's derived quota, never the raw lease or another reservation.
class CaptureImageConsumer {
public:
  Result<SharedCompressedImages> rawAtlas(const CapturedFrame &frame,
                                          std::uint8_t mask = 0xf);
  CaptureImageConsumerReport report() const;

private:
  friend class CaptureImageSharing;
  friend class CaptureOutlet;
  explicit CaptureImageConsumer(
      std::shared_ptr<detail::CaptureImageQuota> quota);
  std::shared_ptr<detail::CaptureImageQuota> quota_;
  std::uint64_t epoch_ = 0;
};
class CaptureImageSharing {
public:
  explicit CaptureImageSharing(std::size_t max_indexed_frames = 32);
  explicit CaptureImageSharing(CaptureImagePoolOptions options);
  ~CaptureImageSharing();
  CaptureImageSharing(const CaptureImageSharing &) = delete;
  CaptureImageSharing &operator=(const CaptureImageSharing &) = delete;
  Result<SharedCompressedImages> rawAtlas(const CapturedFrame &frame,
                                          std::uint8_t mask = 0xf);
  Result<std::shared_ptr<CaptureImageConsumer>>
  consumer(CaptureImageConsumerOptions options = {});
  CaptureImageSharingReport report() const;
  CaptureImageResourceReport resources() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace orbvi_sdk
