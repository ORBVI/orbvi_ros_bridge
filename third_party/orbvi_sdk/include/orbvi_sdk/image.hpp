#pragma once

#include <charconv>
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

struct RectifiedImageIdentity {
  std::uint32_t pair_id = 0;
  std::string side;
  std::string pair_direction;
  std::string source_camera_direction;
};

inline std::string RectifiedPairDirection(std::uint32_t pair_id) {
  static const char* const kPairDirections[] = {
      "front",
      "right",
      "rear",
      "left",
  };
  return pair_id < 4u ? kPairDirections[pair_id] : "";
}

inline std::string RectifiedSourceCameraDirection(
    std::uint32_t pair_id,
    const std::string& side) {
  static const char* const kSourceDirections[] = {
      "front",
      "right",
      "rear",
      "left",
      "front",
  };
  if (pair_id >= 4u) {
    return "";
  }
  if (side == "left") {
    return kSourceDirections[pair_id];
  }
  if (side == "right") {
    return kSourceDirections[pair_id + 1u];
  }
  return "";
}

inline bool IsRectifiedDirectionValue(const std::string& direction) {
  return direction == "front" ||
         direction == "right" ||
         direction == "rear" ||
         direction == "left";
}

// pair_direction/source_camera_direction are additive ORBVI1 metadata. Frames
// from older devices are accepted by deriving the same values from pair_id and
// side. When present, the SDK Core metadata is authoritative.
inline std::optional<RectifiedImageIdentity> ResolveRectifiedImageIdentity(
    const MetadataMap& metadata,
    const std::string& prefix = {}) {
  const auto pair_it = metadata.find(prefix + "pair_id");
  const auto side_it = metadata.find(prefix + "side");
  if (pair_it == metadata.end() || side_it == metadata.end()) {
    return std::nullopt;
  }

  std::uint32_t pair_id = 0;
  const auto parsed = std::from_chars(
      pair_it->second.data(),
      pair_it->second.data() + pair_it->second.size(),
      pair_id);
  if (parsed.ec != std::errc{} ||
      parsed.ptr != pair_it->second.data() + pair_it->second.size() ||
      pair_id >= 4u ||
      (side_it->second != "left" && side_it->second != "right")) {
    return std::nullopt;
  }

  RectifiedImageIdentity identity;
  identity.pair_id = pair_id;
  identity.side = side_it->second;

  const std::string expected_pair_direction =
      RectifiedPairDirection(pair_id);
  const auto pair_direction_it =
      metadata.find(prefix + "pair_direction");
  if (pair_direction_it != metadata.end() &&
      pair_direction_it->second != expected_pair_direction) {
    return std::nullopt;
  }
  const std::string expected_source_direction =
      RectifiedSourceCameraDirection(pair_id, identity.side);
  const auto source_direction_it =
      metadata.find(prefix + "source_camera_direction");
  if (source_direction_it != metadata.end() &&
      source_direction_it->second != expected_source_direction) {
    return std::nullopt;
  }
  identity.pair_direction =
      pair_direction_it != metadata.end()
          ? pair_direction_it->second
          : expected_pair_direction;
  identity.source_camera_direction =
      source_direction_it != metadata.end()
          ? source_direction_it->second
          : expected_source_direction;
  return identity;
}

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
