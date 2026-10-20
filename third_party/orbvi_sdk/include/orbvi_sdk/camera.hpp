#pragma once

#include "orbvi_camera_contract/product.hpp"
#include "orbvi_sdk/image.hpp"

namespace orbvi_sdk {

// Product identity comes from the selected session's reserved host metadata,
// never from the transport, camera number or image layout alone.
Result<orbvi_camera_contract::Product> ResolveCameraProduct(const FrameView& frame);

struct CameraImageIdentity {
  orbvi_camera_contract::Product product = orbvi_camera_contract::Product::Unknown;
  std::uint32_t physical_camera_id = 0;
  // 4P connectors have identity but no mounting direction.
  std::optional<orbvi_camera_contract::Role> direction;
  std::string layout;
  std::string calibration_version;
  std::string calibration_hash;
};

// Resolve an extracted raw image. Missing/unknown/conflicting product identity,
// layout or physical camera ID is an error; no platform fallback is permitted.
// Calibration references are preserved, not evidence of calibrated geometry.
Result<CameraImageIdentity> ResolveCameraImageIdentity(
    const FrameView& frame, const CompressedImage& image);

// Row-major mask (low four bits). Results own losslessly cropped JPEGs; at most
// four 1280x1088 tiles are produced. Does not infer a product from the layout.
Result<CompressedImageSet> ExtractRawAtlasCompressedImages(
    const FrameView& frame, std::uint8_t requested_mask = 0x0f);

}  // namespace orbvi_sdk
