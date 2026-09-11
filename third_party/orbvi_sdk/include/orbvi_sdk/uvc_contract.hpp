#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "orbvi_sdk/frame_sync.hpp"
#include "orbvi_sdk/raw_atlas_contract.hpp"

namespace orbvi_sdk {

// Host-side mirror of orbvi_sdk_core/uvc_contract.hpp. Function IDs and
// profiles are protocol ABI and must remain byte-for-byte equivalent.
constexpr std::uint16_t kUvcProfileContractVersion = 2;

constexpr std::uint32_t MakeUvcFourCc(char a, char b, char c, char d) {
  return static_cast<std::uint32_t>(static_cast<std::uint8_t>(a)) |
         (static_cast<std::uint32_t>(static_cast<std::uint8_t>(b)) << 8u) |
         (static_cast<std::uint32_t>(static_cast<std::uint8_t>(c)) << 16u) |
         (static_cast<std::uint32_t>(static_cast<std::uint8_t>(d)) << 24u);
}

constexpr std::uint32_t kUvcFourCcMjpeg = MakeUvcFourCc('M', 'J', 'P', 'G');

enum class UvcTileOrder : std::uint8_t {
  SingleImage = 0,
  RawFrontRightRearLeft = 1,
  RectifiedPairMajorLeftRight = 2,
};

constexpr std::uint16_t kUvcRawAtlasProfileV1Id = 0x0001;
constexpr std::uint16_t kUvcRawAtlasProfileV2Id = 0x0002;
constexpr std::uint16_t kUvcRectifiedAtlasProfileV1Id = 0x0101;
constexpr std::uint16_t kUvcRectifiedAtlasProfileV2Id = 0x0102;
constexpr std::uint16_t kUvcPanoramaProfileV1Id = 0x0201;
constexpr std::uint16_t kUvcPanoramaProfileV2Id = 0x0202;
constexpr std::uint16_t kUvcBevProfileIdBase = 0x0300;
constexpr std::uint16_t kUvcBevProfileV1Id = 0x0301;

struct UvcFormatProfile {
  FrameSyncUvcIndex function_id = FrameSyncUvcIndex::RawFisheye;
  UvcTileOrder tile_order = UvcTileOrder::SingleImage;
  std::uint16_t format_profile_id = 0;
  std::uint16_t width = 0;
  std::uint16_t height = 0;
  std::uint32_t fourcc = 0;
  std::uint16_t fps_numerator = 0;
  std::uint16_t fps_denominator = 1;
  std::uint16_t tile_width = 0;
  std::uint16_t tile_height = 0;
  std::uint8_t tile_columns = 0;
  std::uint8_t tile_rows = 0;
  std::uint32_t max_frame_bytes = 0;
};

constexpr bool UvcProfileBelongsToFunction(
    std::uint16_t format_profile_id,
    FrameSyncUvcIndex function_id) {
  return static_cast<std::uint8_t>(format_profile_id >> 8u) ==
         static_cast<std::uint8_t>(function_id);
}

constexpr bool IsValidUvcFormatProfile(const UvcFormatProfile& profile) {
  return profile.format_profile_id != 0 &&
         UvcProfileBelongsToFunction(profile.format_profile_id,
                                     profile.function_id) &&
         profile.width != 0 && profile.height != 0 &&
         profile.fourcc != 0 && profile.fps_numerator != 0 &&
         profile.fps_denominator != 0 && profile.tile_width != 0 &&
         profile.tile_height != 0 && profile.tile_columns != 0 &&
         profile.tile_rows != 0 &&
         static_cast<std::uint32_t>(profile.tile_width) *
                 profile.tile_columns ==
             profile.width &&
         static_cast<std::uint32_t>(profile.tile_height) *
                 profile.tile_rows ==
             profile.height &&
         profile.max_frame_bytes != 0;
}

constexpr UvcFormatProfile kUvcRawAtlasProfileV1 = {
    FrameSyncUvcIndex::RawFisheye,
    UvcTileOrder::RawFrontRightRearLeft,
    kUvcRawAtlasProfileV1Id,
    2560,
    2160,
    kUvcFourCcMjpeg,
    20,
    1,
    1280,
    1080,
    2,
    2,
    16u * 1024u * 1024u,
};

constexpr UvcFormatProfile kUvcRawAtlasProfileV2 = {
    FrameSyncUvcIndex::RawFisheye,
    UvcTileOrder::RawFrontRightRearLeft,
    kUvcRawAtlasProfileV2Id,
    static_cast<std::uint16_t>(kRawAtlasWidth),
    static_cast<std::uint16_t>(kRawAtlasHeight),
    kUvcFourCcMjpeg,
    30,
    1,
    static_cast<std::uint16_t>(kRawAtlasTileWidth),
    static_cast<std::uint16_t>(kRawAtlasTileHeight),
    2,
    2,
    16u * 1024u * 1024u,
};

constexpr UvcFormatProfile kUvcRectifiedAtlasProfileV1 = {
    FrameSyncUvcIndex::RectifiedFisheye,
    UvcTileOrder::RectifiedPairMajorLeftRight,
    kUvcRectifiedAtlasProfileV1Id,
    640,
    1024,
    kUvcFourCcMjpeg,
    20,
    1,
    320,
    256,
    2,
    4,
    4u * 1024u * 1024u,
};

constexpr UvcFormatProfile kUvcRectifiedAtlasProfileV2 = {
    FrameSyncUvcIndex::RectifiedFisheye,
    UvcTileOrder::RectifiedPairMajorLeftRight,
    kUvcRectifiedAtlasProfileV2Id,
    1280,
    2048,
    kUvcFourCcMjpeg,
    30,
    1,
    640,
    512,
    2,
    4,
    4u * 1024u * 1024u,
};

constexpr UvcFormatProfile kUvcPanoramaProfileV1 = {
    FrameSyncUvcIndex::Panorama,
    UvcTileOrder::SingleImage,
    kUvcPanoramaProfileV1Id,
    2048,
    1024,
    kUvcFourCcMjpeg,
    20,
    1,
    2048,
    1024,
    1,
    1,
    8u * 1024u * 1024u,
};

constexpr UvcFormatProfile kUvcPanoramaProfileV2 = {
    FrameSyncUvcIndex::Panorama,
    UvcTileOrder::SingleImage,
    kUvcPanoramaProfileV2Id,
    2048,
    1024,
    kUvcFourCcMjpeg,
    30,
    1,
    2048,
    1024,
    1,
    1,
    8u * 1024u * 1024u,
};

constexpr UvcFormatProfile kUvcBevProfileV1 = {
    FrameSyncUvcIndex::Bev,
    UvcTileOrder::SingleImage,
    kUvcBevProfileV1Id,
    1000,
    1000,
    kUvcFourCcMjpeg,
    20,
    1,
    1000,
    1000,
    1,
    1,
    4u * 1024u * 1024u,
};

inline const char* UvcInterfaceName(FrameSyncUvcIndex function_id) {
  switch (function_id) {
    case FrameSyncUvcIndex::RawFisheye:
      return "ORBVI UVC Raw";
    case FrameSyncUvcIndex::RectifiedFisheye:
      return "ORBVI UVC Rectified";
    case FrameSyncUvcIndex::Panorama:
      return "ORBVI UVC Panorama";
    case FrameSyncUvcIndex::Bev:
      return "ORBVI UVC BEV";
  }
  return "";
}

inline bool ParseUvcInterfaceName(
    std::string_view name,
    FrameSyncUvcIndex* function_id) {
  for (std::uint8_t i = 0; i < kFrameSyncUvcCount; ++i) {
    const auto candidate = static_cast<FrameSyncUvcIndex>(i);
    if (name == UvcInterfaceName(candidate)) {
      if (function_id != nullptr) {
        *function_id = candidate;
      }
      return true;
    }
  }
  return false;
}

// Host discovery groups all child interfaces under one USB composite parent,
// identified by (VID, PID, iSerial). Each UVC function is resolved by the exact
// iInterface string above, then bound to both its video and
// V4L2_META_FMT_UVC metadata node. /dev/videoN ordering is never persisted.
struct UsbCompositeSelector {
  std::uint16_t vendor_id = 0;
  std::uint16_t product_id = 0;
  std::string serial_number;
  std::string sysfs_device_path;
};

struct UvcFunctionBinding {
  FrameSyncUvcIndex function_id = FrameSyncUvcIndex::RawFisheye;
  std::uint16_t format_profile_id = 0;
  std::uint8_t usb_interface_number = 0xff;
  std::string interface_name;
  std::string video_node;
  std::string metadata_node;
};

struct VendorBulkBinding {
  std::uint8_t usb_interface_number = 0xff;
  std::uint8_t endpoint_out = 0;
  std::uint8_t endpoint_in = 0;
};

struct UsbCompositeDiscovery {
  UsbCompositeSelector identity;
  std::vector<UvcFunctionBinding> uvc_functions;
  VendorBulkBinding vendor_bulk;
};

inline bool IsValidUvcFunctionBinding(const UvcFunctionBinding& binding) {
  return binding.interface_name == UvcInterfaceName(binding.function_id) &&
         UvcProfileBelongsToFunction(binding.format_profile_id,
                                     binding.function_id) &&
         binding.usb_interface_number != 0xff &&
         !binding.video_node.empty() && !binding.metadata_node.empty();
}

static_assert(IsValidUvcFormatProfile(kUvcRawAtlasProfileV1),
              "invalid raw UVC V1 profile");
static_assert(IsValidUvcFormatProfile(kUvcRawAtlasProfileV2),
              "invalid raw UVC V2 profile");
static_assert(IsValidUvcFormatProfile(kUvcRectifiedAtlasProfileV1),
              "invalid rectified UVC profile");
static_assert(IsValidUvcFormatProfile(kUvcRectifiedAtlasProfileV2),
              "invalid rectified UVC V2 profile");
static_assert(IsValidUvcFormatProfile(kUvcPanoramaProfileV1),
              "invalid panorama UVC profile");
static_assert(IsValidUvcFormatProfile(kUvcPanoramaProfileV2),
              "invalid panorama UVC V2 profile");
static_assert(IsValidUvcFormatProfile(kUvcBevProfileV1),
              "invalid BEV UVC profile");

}  // namespace orbvi_sdk
