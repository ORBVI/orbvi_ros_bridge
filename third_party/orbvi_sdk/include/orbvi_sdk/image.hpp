#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "orbvi_sdk/frame.hpp"

namespace orbvi_sdk {

enum class PixelFormat {
  Bgr888,
};

enum class DecodeMode {
  RawOnly,
  DecodedOnly,
  RawAndDecoded,
};

enum class OverflowPolicy {
  DropOldest,
  SkipDecode,
  Backpressure,
  StopSubscription,
};

struct DecodePolicy {
  DecodeMode mode = DecodeMode::RawOnly;
  PixelFormat output_pixel_format = PixelFormat::Bgr888;
  std::size_t max_decode_queue_depth = 8;
  OverflowPolicy on_overflow = OverflowPolicy::DropOldest;
  std::uint32_t max_decode_latency_ms = 100;
};

struct DecodedImageView {
  PixelFormat pixel_format = PixelFormat::Bgr888;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t stride = 0;
  std::uint64_t timestamp_ns = 0;
  std::string frame_id;
  std::string camera_id;
  std::string calibration_version;
  std::string calibration_hash;
  const std::uint8_t* data = nullptr;
  std::size_t data_size = 0;
};

struct OwnedDecodedImage {
  PixelFormat pixel_format = PixelFormat::Bgr888;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t stride = 0;
  std::uint64_t timestamp_ns = 0;
  std::string frame_id;
  std::string camera_id;
  std::string calibration_version;
  std::string calibration_hash;
  std::vector<std::uint8_t> data;

  DecodedImageView view() const;
};

Result<OwnedDecodedImage> DecodeMjpegToBgr888(const FrameView& frame);
Result<void> DecodeMjpegImagesToBgr888(
    const FrameView& frame,
    std::vector<OwnedDecodedImage>* images);
Result<std::vector<OwnedDecodedImage>> DecodeMjpegImagesToBgr888(const FrameView& frame);
bool DecodePolicyRequiresImage(const DecodePolicy& policy);
const char* ToString(DecodeMode mode);
const char* ToString(OverflowPolicy policy);

}  // namespace orbvi_sdk
