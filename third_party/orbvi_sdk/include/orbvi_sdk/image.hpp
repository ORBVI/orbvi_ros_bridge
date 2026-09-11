#pragma once

#include "orbvi_camera_contract/rig.hpp"
#include <array>
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

inline constexpr std::size_t kRectifiedImageCount = 8;
inline constexpr std::uint8_t kAllRectifiedImagesMask = 0xffU;

inline std::string RectifiedPairDirection(std::uint32_t pair_id) {
  const auto pair = orbvi_camera_contract::kRk3588Rig.pair(pair_id);
  return pair ? orbvi_camera_contract::ToString(pair->direction) : "";
}

inline std::string RectifiedSourceCameraDirection(
    std::uint32_t pair_id, const std::string& side) {
  const auto pair = orbvi_camera_contract::kRk3588Rig.pair(pair_id);
  if (!pair || (side != "left" && side != "right")) return "";
  return orbvi_camera_contract::ToString(orbvi_camera_contract::kRoles[
      side == "left" ? pair->left_index : pair->right_index]);
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
  const auto parsed =
      std::from_chars(pair_it->second.data(),
                      pair_it->second.data() + pair_it->second.size(), pair_id);
  if (parsed.ec != std::errc{} ||
      parsed.ptr != pair_it->second.data() + pair_it->second.size() ||
      pair_id >= 4u ||
      (side_it->second != "left" && side_it->second != "right")) {
    return std::nullopt;
  }

  RectifiedImageIdentity identity;
  identity.pair_id = pair_id;
  identity.side = side_it->second;

  const std::string expected_pair_direction = RectifiedPairDirection(pair_id);
  const auto pair_direction_it = metadata.find(prefix + "pair_direction");
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
  identity.pair_direction = pair_direction_it != metadata.end()
          ? pair_direction_it->second
          : expected_pair_direction;
  identity.source_camera_direction = source_direction_it != metadata.end()
          ? source_direction_it->second
          : expected_source_direction;
  return identity;
}

std::optional<RectifiedImageIdentity>
RectifiedImageIdentityForIndex(std::size_t index);

// A typed compressed logical rectified image. V1 entries borrow bytes from the
// input FrameView and are valid only while that frame payload remains alive.
// V2 entries own the JPEG produced by a coefficient-domain lossless crop.
struct RectifiedCompressedImage {
  RectifiedImageIdentity identity;
  std::uint64_t timestamp_ns = 0;
  std::string frame_id;
  std::string calibration_version;
  std::string calibration_hash;
  std::vector<std::uint8_t> owned_jpeg;
  const std::uint8_t *borrowed_data = nullptr;
  std::size_t borrowed_size = 0;

  const std::uint8_t *jpeg_data() const;
  std::size_t jpeg_size() const;
};

struct RectifiedCompressedImageSet {
  std::array<std::optional<RectifiedCompressedImage>, kRectifiedImageCount>
      images;
};

struct CompressedImage {
  StreamId stream_id = StreamId::Unknown;
  std::string image_id;
  std::optional<RectifiedImageIdentity> rectified_identity;
  std::uint64_t timestamp_ns = 0;
  std::string frame_id;
  std::string calibration_version;
  std::string calibration_hash;
  std::vector<std::uint8_t> owned_jpeg;
  const std::uint8_t *borrowed_data = nullptr;
  std::size_t borrowed_size = 0;

  const std::uint8_t *jpeg_data() const;
  std::size_t jpeg_size() const;
};

struct CompressedImageRequest {
  std::uint8_t rectified_requested_mask = kAllRectifiedImagesMask;
};

struct CompressedImageSet {
  std::vector<CompressedImage> images;
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

// Owns the two decoded 320x1024 physical atlases. views() returns eight
// pair-major, non-owning 320x256 BGR888 ROIs backed by those two atlases;
// callers must not retain a view after this owner is destroyed or moved.
struct OwnedRectifiedAtlasImageSet {
  std::array<OwnedDecodedImage, 2> atlases;

  std::array<DecodedImageView, 8> views() const;
};

Result<OwnedDecodedImage> DecodeMjpegToBgr888(const FrameView& frame);
Result<OwnedRectifiedAtlasImageSet> DecodeRectifiedAtlasBundleToBgr888(
    const FrameView& frame);
Result<RectifiedCompressedImageSet> ExtractRectifiedCompressedImages(
    const FrameView& frame,
    std::uint8_t requested_mask = kAllRectifiedImagesMask);
Result<CompressedImageSet> ExtractCompressedImages(
    const FrameView& frame,
    const CompressedImageRequest& request = {});
Result<void> DecodeMjpegImagesToBgr888(
    const FrameView& frame,
    std::vector<OwnedDecodedImage>* images);
// RAW_ATLAS_MJPEG is decoded once and split into four 1280x1088 images in
// fixed front/right/rear/left camera order (X5 camera IDs 0/3/2/1).
Result<std::vector<OwnedDecodedImage>> DecodeMjpegImagesToBgr888(const FrameView& frame);
bool DecodePolicyRequiresImage(const DecodePolicy& policy);
const char* ToString(DecodeMode mode);
const char* ToString(OverflowPolicy policy);

}  // namespace orbvi_sdk
