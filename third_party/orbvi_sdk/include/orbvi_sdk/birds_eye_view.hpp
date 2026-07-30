#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "orbvi_sdk/fisheye_rig.hpp"
#include "orbvi_sdk/frame.hpp"
#include "orbvi_sdk/image.hpp"

namespace orbvi_sdk {

// Ground-plane bird's-eye-view output generated from the four decoded
// front/right/rear/left fisheye images. The projection implementation is kept
// inside liborbvi_sdk; callers only own the resulting BGR image.
struct BirdsEyeViewOptions {
  double camera_height_m = 1.0;
  double extent_m = 10.0;
  double resolution_m_per_pixel = 0.02;
  std::string frame_id = "base_link";
};

struct BirdsEyeViewImageView {
  PixelFormat pixel_format = PixelFormat::Bgr888;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t stride = 0;
  std::uint64_t timestamp_ns = 0;
  std::string frame_id;
  std::string calibration_version;
  std::string calibration_hash;
  const std::uint8_t* data = nullptr;
  std::size_t data_size = 0;
  const MetadataMap* metadata = nullptr;
};

struct OwnedBirdsEyeViewImage {
  PixelFormat pixel_format = PixelFormat::Bgr888;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t stride = 0;
  std::uint64_t timestamp_ns = 0;
  std::string frame_id;
  std::string calibration_version;
  std::string calibration_hash;
  std::vector<std::uint8_t> data;
  MetadataMap metadata;

  BirdsEyeViewImageView view() const;
};

class BirdsEyeViewGenerator {
 public:
  BirdsEyeViewGenerator();
  ~BirdsEyeViewGenerator();
  BirdsEyeViewGenerator(BirdsEyeViewGenerator&&) noexcept;
  BirdsEyeViewGenerator& operator=(BirdsEyeViewGenerator&&) noexcept;
  BirdsEyeViewGenerator(const BirdsEyeViewGenerator&) = delete;
  BirdsEyeViewGenerator& operator=(const BirdsEyeViewGenerator&) = delete;

  Result<void> Initialize(
      const PanoramaCalibration& calibration,
      const BirdsEyeViewOptions& options = {});
  Result<OwnedBirdsEyeViewImage> Generate(
      const std::vector<DecodedImageView>& images) const;
  Result<OwnedBirdsEyeViewImage> Generate(
      const std::vector<OwnedDecodedImage>& images) const;
  bool initialized() const;
  std::uint32_t width() const;
  std::uint32_t height() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace orbvi_sdk
