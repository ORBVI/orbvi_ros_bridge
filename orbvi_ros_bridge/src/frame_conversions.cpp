#include "frame_conversions.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <type_traits>
#include <utility>

#include "orbvi_sdk/image.hpp"

#include <boost/array.hpp>
#include <livox_ros_driver2/CustomPoint.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace orbvi_ros_bridge {
namespace {

using MetadataMap = orbvi_sdk::MetadataMap;

constexpr std::size_t kImuPayloadDoubleCount = 37;
constexpr std::size_t kVioPayloadDoubleCount = 85;
constexpr std::size_t kLivoxCustomHeaderBytes =
    sizeof(std::uint64_t) + sizeof(std::uint32_t) + sizeof(std::uint8_t) +
    3 * sizeof(std::uint8_t);
constexpr std::size_t kLivoxCustomPointBytes =
    sizeof(std::uint32_t) + 3 * sizeof(float) + 3 * sizeof(std::uint8_t);
constexpr std::uint32_t kDepthPanoramaWidth = 2048;
constexpr std::uint32_t kDepthPanoramaFullHeight = 1024;
constexpr std::uint32_t kDepthPanoramaCropTop = 280;
constexpr std::uint32_t kDepthPanoramaCropBottom = 280;
constexpr double kDepthPanoramaMinDisparityPx = 0.5;
constexpr double kDepthPanoramaMinRangeM = 0.1;
constexpr double kDepthPanoramaMaxRangeM = 20.0;
constexpr const char* kDepthPanoramaFrameId = "orbvi/depth/panorama";
constexpr int kDepthVisualizationGridSize = 5;
constexpr double kPi = 3.141592653589793238462643383279502884;

struct Rgb {
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
};

struct DecodedDisparityLayout {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t tile_count = 0;
  std::uint32_t tile_width = 0;
  std::uint32_t tile_height = 0;
  std::size_t bytes_per_pixel = 0;
  std::size_t stride = 0;
  double decode_scale = 1.0;
  std::string invalid_value;
};

std::string Trim(const std::string& value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

std::string LowerCopy(std::string value) {
  std::transform(
      value.begin(),
      value.end(),
      value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

std::string MetadataValue(
    const MetadataMap& metadata,
    const std::string& key,
    const std::string& fallback = "") {
  const auto it = metadata.find(key);
  return it == metadata.end() ? fallback : it->second;
}

bool ParseU32(const std::string& value, std::uint32_t* out) {
  if (out == nullptr || value.empty()) {
    return false;
  }
  try {
    std::size_t consumed = 0;
    const unsigned long parsed = std::stoul(value, &consumed, 10);
    if (consumed != value.size() ||
        parsed > std::numeric_limits<std::uint32_t>::max()) {
      return false;
    }
    *out = static_cast<std::uint32_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseU64(const std::string& value, std::uint64_t* out) {
  if (out == nullptr || value.empty()) {
    return false;
  }
  try {
    std::size_t consumed = 0;
    const unsigned long long parsed = std::stoull(value, &consumed, 10);
    if (consumed != value.size()) {
      return false;
    }
    *out = static_cast<std::uint64_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseSize(const std::string& value, std::size_t* out) {
  std::uint64_t parsed = 0;
  if (!ParseU64(value, &parsed) ||
      parsed > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    return false;
  }
  *out = static_cast<std::size_t>(parsed);
  return true;
}

double ParseDoubleOr(const std::string& value, double fallback) {
  if (value.empty()) {
    return fallback;
  }
  char* end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  return end == value.c_str() ? fallback : parsed;
}

std::uint32_t MetadataU32(
    const MetadataMap& metadata,
    const std::string& key,
    std::uint32_t fallback) {
  std::uint32_t value = 0;
  return ParseU32(MetadataValue(metadata, key), &value) ? value : fallback;
}

double MetadataDouble(
    const MetadataMap& metadata,
    const std::string& key,
    double fallback) {
  return ParseDoubleOr(MetadataValue(metadata, key), fallback);
}

bool SetError(std::string* error, const std::string& message) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

float ReadFloat32(const std::uint8_t* data) {
  float value = 0.0f;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

std::uint16_t ReadU16Le(const std::uint8_t* data) {
  return static_cast<std::uint16_t>(data[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8u);
}

bool IsInvalidFloatDisparity(float value, const std::string& invalid_value) {
  if (!std::isfinite(value)) {
    return true;
  }
  if (invalid_value == "nan") {
    return false;
  }
  char* end = nullptr;
  const double invalid = std::strtod(invalid_value.c_str(), &end);
  return end != invalid_value.c_str() && value == static_cast<float>(invalid);
}

std::uint64_t SystemNowNs() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

std::string SanitizeTopicToken(std::string value, const std::string& fallback) {
  value = Trim(value);
  if (value.empty()) {
    return fallback;
  }
  for (char& c : value) {
    const bool ok =
        std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-';
    if (!ok) {
      c = '_';
    }
  }
  return value.empty() ? fallback : value;
}

std::string CameraMetadataKey(std::size_t index, const std::string& field) {
  return "camera." + std::to_string(index) + "." + field;
}

std::string RectifiedMetadataKey(std::size_t index, const std::string& field) {
  return "rectified." + std::to_string(index) + "." + field;
}

ros::Time StampFromNs(std::uint64_t timestamp_ns) {
  if (timestamp_ns == 0) {
    timestamp_ns = SystemNowNs();
  }
  const std::uint64_t sec64 = timestamp_ns / 1000000000ULL;
  const std::uint64_t nsec64 = timestamp_ns % 1000000000ULL;
  const std::uint32_t sec =
      sec64 > std::numeric_limits<std::uint32_t>::max()
          ? std::numeric_limits<std::uint32_t>::max()
          : static_cast<std::uint32_t>(sec64);
  return ros::Time(sec, static_cast<std::uint32_t>(nsec64));
}

std_msgs::Header MakeHeader(
    std::uint64_t timestamp_ns,
    const std::string& frame_id) {
  std_msgs::Header header;
  header.stamp = StampFromNs(timestamp_ns);
  header.frame_id = frame_id.empty() ? "orbvi" : frame_id;
  return header;
}

std_msgs::Header MakeHeader(
    const orbvi_sdk::FrameView& frame,
    const std::string& frame_id_override = "",
    std::uint64_t timestamp_override_ns = 0) {
  return MakeHeader(
      timestamp_override_ns == 0 ? frame.timestamp_ns : timestamp_override_ns,
      frame_id_override.empty() ? frame.frame_id : frame_id_override);
}

bool SlicePayload(
    const orbvi_sdk::FrameView& frame,
    std::size_t offset,
    std::size_t size,
    std::vector<std::uint8_t>* out) {
  if (out == nullptr || frame.payload_data == nullptr ||
      offset > frame.payload_size || size > frame.payload_size - offset) {
    return false;
  }
  out->assign(frame.payload_data + offset, frame.payload_data + offset + size);
  return true;
}

template <typename T>
bool ReadPod(
    const std::uint8_t* payload,
    std::size_t payload_size,
    std::size_t* offset,
    T* out) {
  static_assert(std::is_trivially_copyable<T>::value, "payload value must be POD");
  if (payload == nullptr || offset == nullptr || out == nullptr ||
      *offset > payload_size || sizeof(T) > payload_size - *offset) {
    return false;
  }
  std::memcpy(out, payload + *offset, sizeof(T));
  *offset += sizeof(T);
  return true;
}

template <typename T, std::size_t N>
bool ReadPodArray(
    const std::uint8_t* payload,
    std::size_t payload_size,
    std::size_t* offset,
    boost::array<T, N>* out) {
  if (out == nullptr) {
    return false;
  }
  for (std::size_t i = 0; i < N; ++i) {
    if (!ReadPod(payload, payload_size, offset, &((*out)[i]))) {
      return false;
    }
  }
  return true;
}

std::string StreamTopicSuffix(orbvi_sdk::StreamId stream) {
  switch (stream) {
    case orbvi_sdk::StreamId::RawFisheye:
      return "raw/image/compressed";
    case orbvi_sdk::StreamId::RectifiedFisheye:
      return "rectified/image/compressed";
    case orbvi_sdk::StreamId::Imu:
      return "imu";
    case orbvi_sdk::StreamId::LidarPointCloud:
      return "lidar/custom";
    case orbvi_sdk::StreamId::LidarImu:
      return "lidar/imu";
    case orbvi_sdk::StreamId::Disparity:
      return "disparity";
    default:
      return "unknown";
  }
}

bool NormalizeRectifiedSide(std::string side, std::string* out) {
  if (out == nullptr) {
    return false;
  }
  side = LowerCopy(Trim(side));
  if (side == "left" || side == "right") {
    *out = side;
    return true;
  }
  return false;
}

bool IsRectifiedDirection(const std::string& direction) {
  return direction == "front" ||
         direction == "right" ||
         direction == "rear" ||
         direction == "left";
}

std::string RectifiedTopicSuffix(
    const std::string& direction,
    const std::string& side,
    const std::string& leaf) {
  std::string normalized_side;
  if (!IsRectifiedDirection(direction) ||
      !NormalizeRectifiedSide(side, &normalized_side)) {
    return "";
  }
  return "rectified/" + direction + "/" + normalized_side + "/" + leaf;
}

std::string RectifiedFrameId(
    const std::string& direction,
    const std::string& side) {
  std::string normalized_side;
  if (!IsRectifiedDirection(direction) ||
      !NormalizeRectifiedSide(side, &normalized_side)) {
    return "";
  }
  return "orbvi/rectified/" + direction + "/" + normalized_side;
}

std::string DisparityEncoding(
    orbvi_sdk::FrameFormat format,
    const MetadataMap& metadata) {
  const std::string encoding = MetadataValue(metadata, "encoding");
  if (!encoding.empty()) {
    return encoding;
  }
  return format == orbvi_sdk::FrameFormat::Disparity16UQ8 ? "16UC1" : "32FC1";
}

std::string DepthEncoding(orbvi_sdk::DepthPixelFormat format) {
  switch (format) {
    case orbvi_sdk::DepthPixelFormat::Float32Meters:
      return "32FC1";
    case orbvi_sdk::DepthPixelFormat::Uint16Millimeters:
      return "16UC1";
  }
  return "32FC1";
}

bool ReadDepthMeters(
    const orbvi_sdk::DepthMapView& depth,
    std::uint32_t x,
    std::uint32_t y,
    double* out) {
  if (out == nullptr || depth.data == nullptr || x >= depth.width || y >= depth.height) {
    return false;
  }
  const std::size_t bytes_per_pixel =
      depth.pixel_format == orbvi_sdk::DepthPixelFormat::Float32Meters ? 4u : 2u;
  const std::size_t offset =
      static_cast<std::size_t>(y) * depth.stride +
      static_cast<std::size_t>(x) * bytes_per_pixel;
  if (offset > depth.data_size || bytes_per_pixel > depth.data_size - offset) {
    return false;
  }
  if (depth.pixel_format == orbvi_sdk::DepthPixelFormat::Float32Meters) {
    float value = 0.0f;
    std::memcpy(&value, depth.data + offset, sizeof(value));
    *out = static_cast<double>(value);
    return true;
  }
  const auto raw = static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(depth.data[offset]) |
      (static_cast<std::uint16_t>(depth.data[offset + 1]) << 8));
  *out = static_cast<double>(raw) * 0.001;
  return true;
}

bool IsValidDepth(double depth_m, const DepthVisualizationOptions& options) {
  return std::isfinite(depth_m) && depth_m > options.min_depth_m &&
         depth_m <= options.max_depth_m;
}

struct DepthColorRange {
  bool valid = false;
  double min_depth_m = 0.0;
  double max_depth_m = 0.0;
};

DepthColorRange ComputeDepthColorRange(
    const orbvi_sdk::DepthMapView& depth,
    const DepthVisualizationOptions& options,
    std::uint32_t sample_stride) {
  DepthColorRange range;
  const std::uint32_t stride = std::max<std::uint32_t>(1, sample_stride);
  for (std::uint32_t y = 0; y < depth.height; y += stride) {
    for (std::uint32_t x = 0; x < depth.width; x += stride) {
      double depth_m = 0.0;
      if (!ReadDepthMeters(depth, x, y, &depth_m) || !IsValidDepth(depth_m, options)) {
        continue;
      }
      if (!range.valid) {
        range.valid = true;
        range.min_depth_m = depth_m;
        range.max_depth_m = depth_m;
      } else {
        range.min_depth_m = std::min(range.min_depth_m, depth_m);
        range.max_depth_m = std::max(range.max_depth_m, depth_m);
      }
    }
  }
  if (range.valid && range.max_depth_m <= range.min_depth_m) {
    range.max_depth_m = range.min_depth_m + 0.001;
  }
  return range;
}

double Clamp01(double value) {
  return std::max(0.0, std::min(1.0, value));
}

Rgb JetColor(double value) {
  const double t = Clamp01(value);
  Rgb rgb;
  rgb.r = static_cast<std::uint8_t>(255.0 * Clamp01(1.5 - std::abs(4.0 * t - 3.0)));
  rgb.g = static_cast<std::uint8_t>(255.0 * Clamp01(1.5 - std::abs(4.0 * t - 2.0)));
  rgb.b = static_cast<std::uint8_t>(255.0 * Clamp01(1.5 - std::abs(4.0 * t - 1.0)));
  return rgb;
}

Rgb ColorForDepth(double depth_m, const DepthColorRange& range) {
  const double span = std::max(0.001, range.max_depth_m - range.min_depth_m);
  const double normalized_depth = Clamp01((depth_m - range.min_depth_m) / span);
  const double closeness = 1.0 - normalized_depth;
  Rgb rgb = JetColor(closeness);
  return rgb;
}

bool TileForDepthRow(
    const orbvi_sdk::DepthCalibration& calibration,
    std::uint32_t y,
    const orbvi_sdk::DepthTileCalibration** tile,
    std::uint32_t* local_y) {
  if (tile == nullptr || local_y == nullptr ||
      calibration.tile_height == 0 || calibration.tiles.empty()) {
    return false;
  }
  const std::uint32_t tile_index = y / calibration.tile_height;
  if (tile_index >= calibration.tiles.size()) {
    return false;
  }
  const auto& candidate = calibration.tiles[tile_index];
  if (candidate.fx_px <= 0.0 || candidate.width == 0 || candidate.height == 0) {
    return false;
  }
  *tile = &candidate;
  *local_y = y - tile_index * calibration.tile_height;
  return *local_y < candidate.height;
}

bool MakeDisparityLayout(
    const orbvi_sdk::FrameView& frame,
    const orbvi_sdk::DepthCalibration& calibration,
    DecodedDisparityLayout* layout,
    std::string* error) {
  if (layout == nullptr) {
    return SetError(error, "disparity layout output is null");
  }
  if (!orbvi_sdk::IsDisparityFrame(frame)) {
    return SetError(error, "input frame is not disparity");
  }
  if (frame.metadata == nullptr || frame.payload_data == nullptr) {
    return SetError(error, "disparity frame is incomplete");
  }
  if (calibration.tiles.empty()) {
    return SetError(error, "depth calibration is empty");
  }
  const MetadataMap& metadata = *frame.metadata;
  DecodedDisparityLayout parsed;
  parsed.width = MetadataU32(metadata, "width", calibration.width);
  parsed.height = MetadataU32(metadata, "height", calibration.height);
  parsed.tile_count =
      MetadataU32(metadata, "disparity_tile_count", calibration.tile_count);
  parsed.tile_width =
      MetadataU32(metadata, "disparity_tile_width", calibration.tile_width);
  parsed.tile_height =
      MetadataU32(metadata, "disparity_tile_height", calibration.tile_height);
  parsed.bytes_per_pixel =
      frame.format == orbvi_sdk::FrameFormat::Disparity16UQ8 ? 2u : 4u;
  parsed.stride = MetadataU32(
      metadata,
      "stride",
      static_cast<std::uint32_t>(parsed.width * parsed.bytes_per_pixel));
  if (parsed.width == 0 || parsed.height == 0 || parsed.tile_count == 0 ||
      parsed.tile_width == 0 || parsed.tile_height == 0 ||
      parsed.width != calibration.width || parsed.height != calibration.height ||
      parsed.tile_count > calibration.tiles.size() ||
      parsed.stride < static_cast<std::size_t>(parsed.width) * parsed.bytes_per_pixel) {
    return SetError(error, "disparity layout mismatch");
  }
  const std::size_t expected_payload =
      static_cast<std::size_t>(parsed.height - 1u) * parsed.stride +
      static_cast<std::size_t>(parsed.width) * parsed.bytes_per_pixel;
  if (frame.payload_size < expected_payload) {
    return SetError(error, "disparity payload size mismatch");
  }

  parsed.decode_scale = MetadataDouble(metadata, "disparity_decode_scale", 0.0);
  if (parsed.decode_scale <= 0.0) {
    const double disparity_scale = MetadataDouble(metadata, "disparity_scale", 1.0);
    parsed.decode_scale =
        frame.format == orbvi_sdk::FrameFormat::Disparity16UQ8 && disparity_scale > 0.0
            ? 1.0 / disparity_scale
            : 1.0;
  }
  const auto invalid_it = metadata.find("disparity_invalid_value");
  const auto legacy_invalid_it = metadata.find("invalid_value");
  parsed.invalid_value =
      invalid_it != metadata.end()
          ? invalid_it->second
          : (legacy_invalid_it != metadata.end() ? legacy_invalid_it->second : "nan");

  *layout = std::move(parsed);
  return true;
}

bool ReadDisparityPx(
    const orbvi_sdk::FrameView& frame,
    const DecodedDisparityLayout& layout,
    std::uint32_t x,
    std::uint32_t y,
    double* out) {
  if (out == nullptr || x >= layout.width || y >= layout.height ||
      frame.payload_data == nullptr) {
    return false;
  }
  const std::size_t offset =
      static_cast<std::size_t>(y) * layout.stride +
      static_cast<std::size_t>(x) * layout.bytes_per_pixel;
  double disparity_px = 0.0;
  bool valid = true;
  if (frame.format == orbvi_sdk::FrameFormat::Disparity16UQ8) {
    const std::uint16_t raw = ReadU16Le(frame.payload_data + offset);
    valid = layout.invalid_value != "0" || raw != 0;
    disparity_px = static_cast<double>(raw) * layout.decode_scale;
  } else {
    const float raw = ReadFloat32(frame.payload_data + offset);
    valid = !IsInvalidFloatDisparity(raw, layout.invalid_value);
    disparity_px = static_cast<double>(raw);
  }
  if (!valid || !std::isfinite(disparity_px)) {
    return false;
  }
  *out = disparity_px;
  return true;
}

const orbvi_sdk::DepthTileCalibration* FindTileByIndex(
    const orbvi_sdk::DepthCalibration& calibration,
    std::uint32_t tile_index) {
  for (const auto& tile : calibration.tiles) {
    if (tile.tile_index == tile_index) {
      return &tile;
    }
  }
  return nullptr;
}

bool MakeSingleCompressedImage(
    const orbvi_sdk::FrameView& frame,
    const MetadataMap& metadata,
    CompressedImageOutput* out) {
  if (out == nullptr) {
    return false;
  }
  out->message.header = MakeHeader(frame);
  out->message.format = "jpeg";
  if (!SlicePayload(frame, 0, frame.payload_size, &out->message.data)) {
    return false;
  }

  out->topic_suffix = StreamTopicSuffix(frame.stream_id);
  if (frame.stream_id == orbvi_sdk::StreamId::RawFisheye) {
    const std::string camera_id =
        SanitizeTopicToken(MetadataValue(metadata, "camera_id"), "");
    if (!camera_id.empty()) {
      out->topic_suffix = "raw/camera_" + camera_id + "/image/compressed";
    }
  } else if (frame.stream_id == orbvi_sdk::StreamId::RectifiedFisheye) {
    const auto identity =
        orbvi_sdk::ResolveRectifiedImageIdentity(metadata);
    if (!identity.has_value()) {
      return false;
    }
    out->topic_suffix = RectifiedTopicSuffix(
        identity->pair_direction, identity->side, "image/compressed");
    out->message.header.frame_id =
        RectifiedFrameId(identity->pair_direction, identity->side);
    if (out->topic_suffix.empty() || out->message.header.frame_id.empty()) {
      return false;
    }
  }
  return true;
}

bool MakeDisparityRawMat(const orbvi_sdk::FrameView& frame, cv::Mat* raw) {
  if (raw == nullptr || frame.payload_data == nullptr || frame.metadata == nullptr ||
      frame.stream_id != orbvi_sdk::StreamId::Disparity) {
    return false;
  }

  int type = 0;
  std::size_t bytes_per_pixel = 0;
  if (frame.format == orbvi_sdk::FrameFormat::Disparity32F) {
    type = CV_32FC1;
    bytes_per_pixel = sizeof(float);
  } else if (frame.format == orbvi_sdk::FrameFormat::Disparity16UQ8) {
    type = CV_16UC1;
    bytes_per_pixel = sizeof(std::uint16_t);
  } else {
    return false;
  }

  const MetadataMap& metadata = *frame.metadata;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t step = 0;
  if (!ParseU32(MetadataValue(metadata, "width"), &width) ||
      !ParseU32(MetadataValue(metadata, "height"), &height) ||
      !ParseU32(MetadataValue(metadata, "stride"), &step) ||
      width == 0 || height == 0 ||
      step < static_cast<std::uint64_t>(width) * bytes_per_pixel) {
    return false;
  }
  const std::size_t last_row_offset =
      static_cast<std::size_t>(height - 1u) * static_cast<std::size_t>(step);
  const std::size_t expected_payload =
      last_row_offset + static_cast<std::size_t>(width) * bytes_per_pixel;
  if (frame.payload_size < expected_payload) {
    return false;
  }

  *raw = cv::Mat(
      static_cast<int>(height),
      static_cast<int>(width),
      type,
      const_cast<std::uint8_t*>(frame.payload_data),
      static_cast<std::size_t>(step));
  return true;
}

bool MakeDisparityJetMat(const orbvi_sdk::FrameView& frame, cv::Mat* bgr) {
  if (bgr == nullptr) {
    return false;
  }
  cv::Mat raw;
  if (!MakeDisparityRawMat(frame, &raw)) {
    return false;
  }

  cv::Mat viz_u8;
  cv::normalize(raw, viz_u8, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  cv::applyColorMap(viz_u8, *bgr, cv::COLORMAP_JET);
  return !bgr->empty() && bgr->type() == CV_8UC3;
}

void CopyBgrMatToImage(
    const cv::Mat& bgr,
    const std_msgs::Header& header,
    sensor_msgs::Image* out) {
  out->header = header;
  out->height = static_cast<std::uint32_t>(bgr.rows);
  out->width = static_cast<std::uint32_t>(bgr.cols);
  out->encoding = "bgr8";
  out->is_bigendian = 0;
  out->step = out->width * 3u;
  out->data.resize(static_cast<std::size_t>(out->step) * out->height);
  for (std::uint32_t y = 0; y < out->height; ++y) {
    const auto* row = bgr.ptr<std::uint8_t>(static_cast<int>(y));
    std::copy(row, row + out->step, out->data.data() + static_cast<std::size_t>(y) * out->step);
  }
}

std::size_t BytesPerPixelForEncoding(const std::string& encoding) {
  if (encoding == "bgr8" || encoding == "rgb8") {
    return 3u;
  }
  if (encoding == "32FC1") {
    return 4u;
  }
  if (encoding == "16UC1") {
    return 2u;
  }
  return 0u;
}

bool SliceImageRows(
    const sensor_msgs::Image& source,
    std::uint32_t y0,
    std::uint32_t width,
    std::uint32_t height,
    const std::string& frame_id,
    sensor_msgs::Image* out) {
  if (out == nullptr || source.data.empty() || width == 0 || height == 0 ||
      y0 > source.height || height > source.height - y0 || width > source.width) {
    return false;
  }
  const std::size_t bytes_per_pixel = BytesPerPixelForEncoding(source.encoding);
  if (bytes_per_pixel == 0 && width != source.width) {
    return false;
  }
  const std::uint32_t row_bytes =
      bytes_per_pixel == 0
          ? source.step
          : static_cast<std::uint32_t>(width * bytes_per_pixel);
  if (source.step < row_bytes) {
    return false;
  }
  const std::size_t required_size =
      static_cast<std::size_t>(source.step) * static_cast<std::size_t>(source.height);
  if (source.data.size() < required_size) {
    return false;
  }

  out->header = source.header;
  out->header.frame_id = frame_id;
  out->height = height;
  out->width = width;
  out->encoding = source.encoding;
  out->is_bigendian = source.is_bigendian;
  out->step = row_bytes;
  out->data.resize(static_cast<std::size_t>(row_bytes) * height);
  for (std::uint32_t row = 0; row < height; ++row) {
    const auto src_offset =
        static_cast<std::size_t>(y0 + row) * static_cast<std::size_t>(source.step);
    const auto dst_offset = static_cast<std::size_t>(row) * row_bytes;
    std::copy(
        source.data.data() + src_offset,
        source.data.data() + src_offset + row_bytes,
        out->data.data() + dst_offset);
  }
  return true;
}

std::string FormatDepthMeters(double depth_m) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << depth_m << "m";
  return stream.str();
}

bool AnnotateDepthGrid(
    const orbvi_sdk::DepthMapView& depth,
    const orbvi_sdk::DepthCalibration& calibration,
    const DepthVisualizationOptions& options,
    sensor_msgs::Image* image) {
  if (!options.show_grid_values) {
    return true;
  }
  if (image == nullptr || image->encoding != "bgr8" || image->data.empty() ||
      image->width != depth.width || image->height != depth.height ||
      image->step < image->width * 3u) {
    return false;
  }

  const std::size_t required_size =
      static_cast<std::size_t>(image->step) * image->height;
  if (image->data.size() < required_size) {
    return false;
  }

  cv::Mat bgr(
      static_cast<int>(image->height),
      static_cast<int>(image->width),
      CV_8UC3,
      image->data.data(),
      static_cast<std::size_t>(image->step));
  const cv::Scalar white(255, 255, 255);
  const cv::Scalar black(0, 0, 0);

  for (const auto& tile : calibration.tiles) {
    const std::uint32_t y0 = tile.tile_index * calibration.tile_height;
    if (tile.width == 0 || tile.height == 0 ||
        tile.width > depth.width || y0 >= depth.height ||
        tile.height > depth.height - y0) {
      continue;
    }

    const int grid_thickness = std::clamp(
        static_cast<int>(std::lround(
            static_cast<double>(std::min(tile.width, tile.height)) / 400.0)),
        1,
        2);
    for (int index = 0; index < kDepthVisualizationGridSize; ++index) {
      const int x = static_cast<int>(
          static_cast<std::uint64_t>(index + 1) * tile.width /
          (kDepthVisualizationGridSize + 1));
      const int y = static_cast<int>(
          y0 + static_cast<std::uint64_t>(index + 1) * tile.height /
                   (kDepthVisualizationGridSize + 1));
      cv::line(
          bgr,
          cv::Point(x, static_cast<int>(y0)),
          cv::Point(x, static_cast<int>(y0 + tile.height - 1u)),
          white,
          grid_thickness,
          cv::LINE_8);
      cv::line(
          bgr,
          cv::Point(0, y),
          cv::Point(static_cast<int>(tile.width - 1u), y),
          white,
          grid_thickness,
          cv::LINE_8);
    }

    int reference_baseline = 0;
    const cv::Size reference_size = cv::getTextSize(
        "20.00m", cv::FONT_HERSHEY_SIMPLEX, 1.0, 1, &reference_baseline);
    const double grid_spacing_x = static_cast<double>(tile.width) /
                                  (kDepthVisualizationGridSize + 1);
    const double grid_spacing_y = static_cast<double>(tile.height) /
                                  (kDepthVisualizationGridSize + 1);
    const double width_scale =
        (grid_spacing_x - 6.0) / std::max(1, reference_size.width);
    const double height_scale =
        (grid_spacing_y - 6.0) /
        std::max(1, reference_size.height + reference_baseline);
    const double font_scale = std::clamp(
        std::min(width_scale, height_scale), 0.25, 0.8);
    const int text_thickness = font_scale >= 0.65 ? 2 : 1;
    const int text_offset = std::max(3, grid_thickness * 2);

    for (int row = 0; row < kDepthVisualizationGridSize; ++row) {
      const std::uint32_t sample_y =
          y0 + static_cast<std::uint64_t>(row + 1) * tile.height /
                   (kDepthVisualizationGridSize + 1);
      for (int column = 0; column < kDepthVisualizationGridSize; ++column) {
        const std::uint32_t sample_x =
            static_cast<std::uint64_t>(column + 1) * tile.width /
            (kDepthVisualizationGridSize + 1);

        double depth_m = 0.0;
        const std::string label =
            ReadDepthMeters(depth, sample_x, sample_y, &depth_m) &&
                    IsValidDepth(depth_m, options)
                ? FormatDepthMeters(depth_m)
                : "--";
        int baseline = 0;
        const cv::Size text_size = cv::getTextSize(
            label,
            cv::FONT_HERSHEY_SIMPLEX,
            font_scale,
            text_thickness,
            &baseline);
        const int text_x = std::min(
            static_cast<int>(tile.width) - text_size.width - 1,
            static_cast<int>(sample_x) + text_offset);
        const int text_y = std::max(
            static_cast<int>(y0) + text_size.height + 1,
            static_cast<int>(sample_y) - text_offset);
        const cv::Point origin(text_x, text_y);
        cv::putText(
            bgr,
            label,
            origin,
            cv::FONT_HERSHEY_SIMPLEX,
            font_scale,
            black,
            text_thickness + 2,
            cv::LINE_AA);
        cv::putText(
            bgr,
            label,
            origin,
            cv::FONT_HERSHEY_SIMPLEX,
            font_scale,
            white,
            text_thickness,
            cv::LINE_AA);
      }
    }
  }
  return true;
}

std::vector<ImageOutput> SplitVerticalTileImage(
    const sensor_msgs::Image& source,
    const orbvi_sdk::DepthCalibration& calibration,
    const std::string& leaf) {
  std::vector<ImageOutput> outputs;
  outputs.reserve(calibration.tiles.size());
  for (const auto& tile : calibration.tiles) {
    const std::string direction =
        orbvi_sdk::RectifiedPairDirection(tile.tile_index);
    if (direction.empty()) {
      continue;
    }
    const std::uint32_t y0 = tile.tile_index * calibration.tile_height;
    ImageOutput output;
    output.topic_suffix = direction + "/" + leaf;
    if (SliceImageRows(
            source,
            y0,
            tile.width,
            tile.height,
            "orbvi/" + direction + "/" + leaf,
            &output.message)) {
      outputs.push_back(std::move(output));
    }
  }
  return outputs;
}

}  // namespace

std::vector<std::string> SplitCsv(const std::string& value) {
  std::vector<std::string> out;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ',')) {
    item = Trim(item);
    if (!item.empty()) {
      out.push_back(item);
    }
  }
  return out;
}

std::string NormalizeTopicPrefix(std::string prefix) {
  prefix = Trim(prefix);
  while (prefix.size() > 1 && prefix.back() == '/') {
    prefix.pop_back();
  }
  if (prefix.empty()) {
    return "";
  }
  if (prefix.front() != '/') {
    prefix.insert(prefix.begin(), '/');
  }
  return prefix;
}

std::string JoinTopic(const std::string& prefix, std::string suffix) {
  while (!suffix.empty() && suffix.front() == '/') {
    suffix.erase(suffix.begin());
  }
  if (prefix.empty()) {
    return "/" + suffix;
  }
  return suffix.empty() ? prefix : prefix + "/" + suffix;
}

bool IsBridgeImageStream(orbvi_sdk::StreamId stream) {
  return stream == orbvi_sdk::StreamId::RawFisheye ||
         stream == orbvi_sdk::StreamId::RectifiedFisheye;
}

bool IsBridgePanoramaToken(const std::string& input) {
  std::string name = LowerCopy(Trim(input));
  std::replace(name.begin(), name.end(), '-', '_');
  return name == "pano" || name == "panorama" ||
         name == "pano_display" || name == "pano_display_stream" ||
         name == "panorama_stream";
}

bool ParseBridgeStream(const std::string& input, orbvi_sdk::StreamId* out) {
  if (out == nullptr) {
    return false;
  }
  std::string name = LowerCopy(Trim(input));
  std::replace(name.begin(), name.end(), '-', '_');
  if (name == "raw") {
    name = "raw_fisheye_stream";
  } else if (name == "rectified") {
    name = "rectified_fisheye_stream";
  } else if (name == "imu") {
    name = "imu_stream";
  } else if (name == "lidar") {
    name = "lidar_pointcloud_stream";
  } else if (name == "lidar_imu") {
    name = "lidar_imu_stream";
  } else if (name == "disparity") {
    name = "disparity_stream";
  } else if (name == "vio") {
    name = "vio_pose_stream";
  } else if (IsBridgePanoramaToken(name)) {
    *out = orbvi_sdk::StreamId::Unknown;
    return false;
  }
  return orbvi_sdk::ParseStreamId(name, out);
}

std::vector<CompressedImageOutput> MakeCompressedImageMessages(
    const orbvi_sdk::FrameView& frame,
    bool split_bundles,
    std::string* skipped_reason) {
  const MetadataMap empty_metadata;
  const MetadataMap& metadata = frame.metadata != nullptr ? *frame.metadata : empty_metadata;
  const std::string bundle_type = MetadataValue(metadata, "bundle_type");
  std::vector<CompressedImageOutput> outputs;

  if (bundle_type == "RAW_MJPEG_BUNDLE") {
    if (!split_bundles) {
      if (skipped_reason != nullptr) {
        *skipped_reason = "multi-image raw frame cannot be published as one valid CompressedImage";
      }
      return outputs;
    }
    std::uint32_t count = 0;
    if (!ParseU32(MetadataValue(metadata, "camera_count"), &count) &&
        !ParseU32(MetadataValue(metadata, "entry_count"), &count)) {
      return outputs;
    }
    for (std::uint32_t i = 0; i < count; ++i) {
      std::size_t offset = 0;
      std::size_t size = 0;
      if (!ParseSize(MetadataValue(metadata, CameraMetadataKey(i, "payload_offset")), &offset) ||
          !ParseSize(MetadataValue(metadata, CameraMetadataKey(i, "payload_size")), &size)) {
        continue;
      }
      CompressedImageOutput output;
      std::uint64_t timestamp_ns = 0;
      ParseU64(MetadataValue(metadata, CameraMetadataKey(i, "timestamp_ns")), &timestamp_ns);
      output.message.header = MakeHeader(
          frame,
          MetadataValue(metadata, CameraMetadataKey(i, "source_frame_id"), frame.frame_id),
          timestamp_ns);
      output.message.format = "jpeg";
      if (!SlicePayload(frame, offset, size, &output.message.data)) {
        continue;
      }
      const std::string camera_id = SanitizeTopicToken(
          MetadataValue(metadata, CameraMetadataKey(i, "camera_id")),
          std::to_string(i));
      output.topic_suffix = "raw/camera_" + camera_id + "/image/compressed";
      outputs.push_back(std::move(output));
    }
    return outputs;
  }

  if (bundle_type == "RECTIFIED_FISHEYE_BUNDLE") {
    if (!split_bundles) {
      if (skipped_reason != nullptr) {
        *skipped_reason = "multi-image rectified frame cannot be published as one valid CompressedImage";
      }
      return outputs;
    }
    std::uint32_t count = 0;
    if (!ParseU32(MetadataValue(metadata, "image_count"), &count) &&
        !ParseU32(MetadataValue(metadata, "entry_count"), &count)) {
      return outputs;
    }
    for (std::uint32_t i = 0; i < count; ++i) {
      std::size_t offset = 0;
      std::size_t size = 0;
      if (!ParseSize(MetadataValue(metadata, RectifiedMetadataKey(i, "payload_offset")), &offset) ||
          !ParseSize(MetadataValue(metadata, RectifiedMetadataKey(i, "payload_size")), &size)) {
        continue;
      }
      CompressedImageOutput output;
      std::uint64_t timestamp_ns = 0;
      ParseU64(MetadataValue(metadata, RectifiedMetadataKey(i, "timestamp_ns")), &timestamp_ns);
      output.message.header = MakeHeader(frame, frame.frame_id, timestamp_ns);
      output.message.format = "jpeg";
      if (!SlicePayload(frame, offset, size, &output.message.data)) {
        continue;
      }
      const auto identity = orbvi_sdk::ResolveRectifiedImageIdentity(
          metadata, "rectified." + std::to_string(i) + ".");
      if (identity.has_value()) {
        output.topic_suffix = RectifiedTopicSuffix(
            identity->pair_direction,
            identity->side,
            "image/compressed");
        output.message.header.frame_id =
            RectifiedFrameId(identity->pair_direction, identity->side);
      }
      if (output.topic_suffix.empty() || output.message.header.frame_id.empty()) {
        continue;
      }
      outputs.push_back(std::move(output));
    }
    return outputs;
  }

  CompressedImageOutput output;
  if (MakeSingleCompressedImage(frame, metadata, &output)) {
    outputs.push_back(std::move(output));
  }
  return outputs;
}

std::vector<ImageOutput> MakeDecodedImageMessages(
    orbvi_sdk::StreamId stream,
    const orbvi_sdk::FrameDelivery& delivery) {
  std::vector<ImageOutput> outputs;
  const auto append = [&](const orbvi_sdk::DecodedImageView& decoded, std::size_t index) {
    if (decoded.data == nullptr || decoded.data_size == 0 ||
        decoded.width == 0 || decoded.height == 0 || decoded.stride == 0) {
      return;
    }
    ImageOutput output;
    output.message.header = MakeHeader(decoded.timestamp_ns, decoded.frame_id);
    output.message.height = decoded.height;
    output.message.width = decoded.width;
    output.message.encoding = "bgr8";
    output.message.is_bigendian = 0;
    output.message.step = decoded.stride;
    output.message.data.assign(decoded.data, decoded.data + decoded.data_size);
    const std::string token = SanitizeTopicToken(decoded.camera_id, std::to_string(index));
    if (stream == orbvi_sdk::StreamId::RawFisheye) {
      output.topic_suffix = "raw/camera_" + token + "/image";
    } else {
      const std::string prefix =
          delivery.decoded_images.size() > 1u
              ? "rectified." + std::to_string(index) + "."
              : "";
      const auto identity = orbvi_sdk::ResolveRectifiedImageIdentity(
          delivery.frame.metadata, prefix);
      if (!identity.has_value()) {
        return;
      }
      output.topic_suffix = RectifiedTopicSuffix(
          identity->pair_direction, identity->side, "image");
      output.message.header.frame_id =
          RectifiedFrameId(identity->pair_direction, identity->side);
      if (output.topic_suffix.empty() || output.message.header.frame_id.empty()) {
        return;
      }
    }
    outputs.push_back(std::move(output));
  };

  for (std::size_t i = 0; i < delivery.decoded_images.size(); ++i) {
    append(delivery.decoded_images[i].view(), i);
  }
  if (outputs.empty() && delivery.decoded_image) {
    append(delivery.decoded_image->view(), 0);
  }
  return outputs;
}

bool MakePanoramaImageMessage(
    const orbvi_sdk::PanoramaImageView& panorama,
    sensor_msgs::Image* out) {
  if (out == nullptr || panorama.data == nullptr ||
      panorama.data_size == 0 || panorama.width == 0 ||
      panorama.height == 0 || panorama.stride == 0 ||
      panorama.stride < panorama.width * 3u) {
    return false;
  }
  const std::size_t expected_size =
      static_cast<std::size_t>(panorama.stride) * panorama.height;
  if (panorama.data_size < expected_size) {
    return false;
  }
  out->header = MakeHeader(
      panorama.timestamp_ns,
      panorama.frame_id.empty() ? "orbvi/panorama/equirectangular" : panorama.frame_id);
  out->height = panorama.height;
  out->width = panorama.width;
  out->encoding = "bgr8";
  out->is_bigendian = 0;
  out->step = panorama.stride;
  out->data.assign(panorama.data, panorama.data + expected_size);
  return true;
}

bool MakeDisparityImage(const orbvi_sdk::FrameView& frame, sensor_msgs::Image* out) {
  if (out == nullptr || frame.payload_data == nullptr) {
    return false;
  }
  const MetadataMap empty_metadata;
  const MetadataMap& metadata = frame.metadata != nullptr ? *frame.metadata : empty_metadata;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t step = 0;
  if (!ParseU32(MetadataValue(metadata, "width"), &width) ||
      !ParseU32(MetadataValue(metadata, "height"), &height) ||
      !ParseU32(MetadataValue(metadata, "stride"), &step)) {
    return false;
  }

  out->header = MakeHeader(frame);
  out->height = height;
  out->width = width;
  out->encoding = DisparityEncoding(frame.format, metadata);
  out->is_bigendian = 0;
  out->step = step;
  return SlicePayload(frame, 0, frame.payload_size, &out->data);
}

std::vector<ImageOutput> MakeSplitDisparityImages(
    const orbvi_sdk::FrameView& frame,
    const orbvi_sdk::DepthCalibration& calibration) {
  sensor_msgs::Image image;
  if (!MakeDisparityImage(frame, &image)) {
    return {};
  }
  return SplitVerticalTileImage(image, calibration, "disparity");
}

bool MakeDepthImage(const orbvi_sdk::DepthMapView& depth, sensor_msgs::Image* out) {
  if (out == nullptr || depth.data == nullptr ||
      depth.width == 0 || depth.height == 0 || depth.stride == 0) {
    return false;
  }
  const std::size_t expected_size =
      static_cast<std::size_t>(depth.stride) * static_cast<std::size_t>(depth.height);
  if (depth.data_size < expected_size) {
    return false;
  }

  out->header = MakeHeader(depth.timestamp_ns, depth.frame_id);
  out->height = depth.height;
  out->width = depth.width;
  out->encoding = DepthEncoding(depth.pixel_format);
  out->is_bigendian = 0;
  out->step = depth.stride;
  out->data.assign(depth.data, depth.data + expected_size);
  return true;
}

std::vector<ImageOutput> MakeSplitDepthImages(
    const orbvi_sdk::DepthMapView& depth,
    const orbvi_sdk::DepthCalibration& calibration) {
  sensor_msgs::Image image;
  if (!MakeDepthImage(depth, &image)) {
    return {};
  }
  return SplitVerticalTileImage(image, calibration, "depth");
}

static orbvi_sdk::DepthPanoramaOptions MakePublishedDepthPanoramaOptions() {
  orbvi_sdk::DepthPanoramaOptions options;
  options.width = kDepthPanoramaWidth;
  options.height = kDepthPanoramaFullHeight;
  options.crop_top = kDepthPanoramaCropTop;
  options.crop_bottom = kDepthPanoramaCropBottom;
  options.min_disparity_px = kDepthPanoramaMinDisparityPx;
  options.min_range_m = kDepthPanoramaMinRangeM;
  options.max_range_m = kDepthPanoramaMaxRangeM;
  options.frame_id = kDepthPanoramaFrameId;
  return options;
}

bool BuildDepthPanoramaLookup(
    const orbvi_sdk::DepthCalibration& calibration,
    orbvi_sdk::DepthPanoramaLookup* out,
    std::string* error) {
  if (out == nullptr) {
    return SetError(error, "depth panorama lookup output is null");
  }
  auto lookup = orbvi_sdk::BuildDepthPanoramaLookup(
      calibration, MakePublishedDepthPanoramaOptions());
  if (!lookup) {
    return SetError(error, lookup.error().message);
  }
  *out = std::move(lookup.value());
  return true;
}

bool MakeDepthPanoramaImage(
    const orbvi_sdk::FrameView& disparity_frame,
    const orbvi_sdk::DepthCalibration& calibration,
    const orbvi_sdk::DepthPanoramaLookup& lookup,
    sensor_msgs::Image* out,
    std::string* error) {
  auto generated =
      orbvi_sdk::GenerateDepthPanorama(
          disparity_frame,
          calibration,
          lookup,
          MakePublishedDepthPanoramaOptions());
  if (!generated) {
    return SetError(error, generated.error().message);
  }
  if (!MakeDepthImage(generated.value().view(), out)) {
    return SetError(error, "failed to convert inverse-lookup depth panorama");
  }
  return true;
}

bool MakeDepthPanoramaImage(
    const orbvi_sdk::RigDepthPanorama& depth,
    sensor_msgs::Image* out,
    std::string* error) {
  if (out == nullptr) {
    return SetError(error, "depth panorama image output is null");
  }
  const std::size_t pixel_count =
      static_cast<std::size_t>(depth.width) * depth.height;
  if (depth.width == 0 || depth.height == 0 ||
      depth.range_m.size() != pixel_count ||
      depth.crop_top + depth.crop_bottom >= depth.full_height ||
      depth.height != depth.full_height - depth.crop_top - depth.crop_bottom) {
    return SetError(error, "rig depth panorama layout is invalid");
  }

  out->header = MakeHeader(depth.timestamp_ns, kDepthPanoramaFrameId);
  out->height = depth.height;
  out->width = depth.width;
  out->encoding = "32FC1";
  out->is_bigendian = 0;
  out->step = depth.width * sizeof(float);
  out->data.resize(pixel_count * sizeof(float));

  std::memcpy(out->data.data(), depth.range_m.data(), out->data.size());
  return true;
}

bool MakeDepthPanoramaVisualizationImage(
    const sensor_msgs::Image& depth_panorama,
    sensor_msgs::Image* out) {
  if (out == nullptr || depth_panorama.encoding != "32FC1" ||
      depth_panorama.width == 0 || depth_panorama.height == 0 ||
      depth_panorama.step < depth_panorama.width * sizeof(float)) {
    return false;
  }
  const std::size_t required_size =
      static_cast<std::size_t>(depth_panorama.step) * depth_panorama.height;
  if (depth_panorama.data.size() < required_size) {
    return false;
  }

  out->header = depth_panorama.header;
  out->height = depth_panorama.height;
  out->width = depth_panorama.width;
  out->encoding = "bgr8";
  out->is_bigendian = 0;
  out->step = depth_panorama.width * 3u;
  out->data.assign(static_cast<std::size_t>(out->step) * out->height, 0);

  std::vector<float> inverse_depth(static_cast<std::size_t>(depth_panorama.width) *
                                   depth_panorama.height, 0.0f);
  float min_inverse = std::numeric_limits<float>::infinity();
  float max_inverse = 0.0f;
  for (std::uint32_t y = 0; y < depth_panorama.height; ++y) {
    for (std::uint32_t x = 0; x < depth_panorama.width; ++x) {
      const std::size_t input_offset =
          static_cast<std::size_t>(y) * depth_panorama.step +
          static_cast<std::size_t>(x) * sizeof(float);
      float range_m = 0.0f;
      std::memcpy(&range_m, depth_panorama.data.data() + input_offset, sizeof(range_m));
      if (!std::isfinite(range_m) || range_m <= 0.0f) {
        continue;
      }
      const float inv = 1.0f / range_m;
      inverse_depth[static_cast<std::size_t>(y) * depth_panorama.width + x] = inv;
      min_inverse = std::min(min_inverse, inv);
      max_inverse = std::max(max_inverse, inv);
    }
  }

  if (!std::isfinite(min_inverse) || max_inverse <= 0.0f) {
    return true;
  }

  const double span = static_cast<double>(max_inverse) - static_cast<double>(min_inverse);
  for (std::uint32_t y = 0; y < depth_panorama.height; ++y) {
    for (std::uint32_t x = 0; x < depth_panorama.width; ++x) {
      const float inv = inverse_depth[static_cast<std::size_t>(y) * depth_panorama.width + x];
      if (inv <= 0.0f) {
        continue;
      }
      const double normalized =
          span > 1e-12 ? (static_cast<double>(inv) - min_inverse) / span : 1.0;
      const Rgb rgb = JetColor(normalized);
      const std::size_t output_offset =
          static_cast<std::size_t>(y) * out->step + static_cast<std::size_t>(x) * 3u;
      out->data[output_offset + 0] = rgb.b;
      out->data[output_offset + 1] = rgb.g;
      out->data[output_offset + 2] = rgb.r;
    }
  }
  return true;
}

bool MakeDepthVisualizationImage(
    const orbvi_sdk::DepthMapView& depth,
    const DepthVisualizationOptions& options,
    sensor_msgs::Image* out) {
  if (out == nullptr || depth.data == nullptr ||
      depth.width == 0 || depth.height == 0 || depth.stride == 0 ||
      options.max_depth_m <= options.min_depth_m) {
    return false;
  }

  out->header = MakeHeader(depth.timestamp_ns, depth.frame_id);
  out->height = depth.height;
  out->width = depth.width;
  out->encoding = "bgr8";
  out->is_bigendian = 0;
  out->step = depth.width * 3u;
  out->data.assign(static_cast<std::size_t>(out->step) * out->height, 0);

  const DepthColorRange color_range = ComputeDepthColorRange(depth, options, 1);
  if (!color_range.valid) {
    return true;
  }
  for (std::uint32_t y = 0; y < depth.height; ++y) {
    for (std::uint32_t x = 0; x < depth.width; ++x) {
      double depth_m = 0.0;
      if (!ReadDepthMeters(depth, x, y, &depth_m) || !IsValidDepth(depth_m, options)) {
        continue;
      }
      const Rgb rgb = ColorForDepth(depth_m, color_range);
      const std::size_t offset =
          static_cast<std::size_t>(y) * out->step + static_cast<std::size_t>(x) * 3u;
      out->data[offset + 0] = rgb.b;
      out->data[offset + 1] = rgb.g;
      out->data[offset + 2] = rgb.r;
    }
  }
  return true;
}

bool MakeDepthVisualizationImage(
    const orbvi_sdk::DepthMapView& depth,
    const orbvi_sdk::FrameView& source_disparity,
    const DepthVisualizationOptions& /*options*/,
    sensor_msgs::Image* out) {
  if (out == nullptr || depth.width == 0 || depth.height == 0) {
    return false;
  }
  cv::Mat viz_color;
  if (!MakeDisparityJetMat(source_disparity, &viz_color) ||
      viz_color.cols != static_cast<int>(depth.width) ||
      viz_color.rows != static_cast<int>(depth.height)) {
    return false;
  }

  CopyBgrMatToImage(viz_color, MakeHeader(source_disparity), out);
  return true;
}

std::vector<ImageOutput> MakeSplitDepthVisualizationImages(
    const orbvi_sdk::DepthMapView& depth,
    const orbvi_sdk::FrameView& source_disparity,
    const DepthVisualizationOptions& options,
    const orbvi_sdk::DepthCalibration& calibration) {
  sensor_msgs::Image image;
  if (!MakeDepthVisualizationImage(depth, source_disparity, options, &image)) {
    return {};
  }
  if (!AnnotateDepthGrid(depth, calibration, options, &image)) {
    return {};
  }
  return SplitVerticalTileImage(image, calibration, "depth/viz");
}

bool MakeDepthPointCloud(
    const orbvi_sdk::DepthMapView& depth,
    const orbvi_sdk::DepthCalibration& calibration,
    const DepthVisualizationOptions& options,
    sensor_msgs::PointCloud2* out) {
  if (out == nullptr || depth.data == nullptr ||
      depth.width == 0 || depth.height == 0 || depth.stride == 0 ||
      options.max_depth_m <= options.min_depth_m) {
    return false;
  }
  const std::uint32_t stride = std::max<std::uint32_t>(1, options.pointcloud_stride);
  const DepthColorRange color_range = ComputeDepthColorRange(depth, options, stride);
  if (!color_range.valid) {
    return false;
  }
  std::size_t point_count = 0;
  for (std::uint32_t y = 0; y < depth.height; y += stride) {
    const orbvi_sdk::DepthTileCalibration* tile = nullptr;
    std::uint32_t local_y = 0;
    if (!TileForDepthRow(calibration, y, &tile, &local_y)) {
      continue;
    }
    for (std::uint32_t x = 0; x < depth.width; x += stride) {
      double depth_m = 0.0;
      if (x < tile->width && ReadDepthMeters(depth, x, y, &depth_m) &&
          IsValidDepth(depth_m, options)) {
        ++point_count;
      }
    }
  }
  if (point_count == 0) {
    return false;
  }

  out->header = MakeHeader(depth.timestamp_ns, depth.frame_id);
  out->height = 1;
  out->width = static_cast<std::uint32_t>(
      std::min<std::size_t>(point_count, std::numeric_limits<std::uint32_t>::max()));
  out->is_bigendian = false;
  out->is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(*out);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(point_count);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*out, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*out, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*out, "z");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> iter_r(*out, "r");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> iter_g(*out, "g");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> iter_b(*out, "b");

  for (std::uint32_t y = 0; y < depth.height; y += stride) {
    const orbvi_sdk::DepthTileCalibration* tile = nullptr;
    std::uint32_t local_y = 0;
    if (!TileForDepthRow(calibration, y, &tile, &local_y)) {
      continue;
    }
    for (std::uint32_t x = 0; x < depth.width; x += stride) {
      double depth_m = 0.0;
      if (x >= tile->width || !ReadDepthMeters(depth, x, y, &depth_m) ||
          !IsValidDepth(depth_m, options)) {
        continue;
      }
      *iter_x = static_cast<float>((static_cast<double>(x) - tile->cx_px) * depth_m / tile->fx_px);
      const double fy_px = tile->fy_px > 0.0 ? tile->fy_px : tile->fx_px;
      *iter_y =
          static_cast<float>((static_cast<double>(local_y) - tile->cy_px) * depth_m / fy_px);
      *iter_z = static_cast<float>(depth_m);
      const Rgb rgb = ColorForDepth(depth_m, color_range);
      *iter_r = rgb.r;
      *iter_g = rgb.g;
      *iter_b = rgb.b;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_r;
      ++iter_g;
      ++iter_b;
    }
  }
  return true;
}

bool MakePointCloudMessage(
    const orbvi_sdk::PointCloud& cloud,
    sensor_msgs::PointCloud2* out) {
  if (out == nullptr) {
    return false;
  }

  out->header = MakeHeader(cloud.timestamp_ns, cloud.frame_id);
  out->height = 1;
  out->width = static_cast<std::uint32_t>(
      std::min<std::size_t>(cloud.points.size(), std::numeric_limits<std::uint32_t>::max()));
  out->is_bigendian = false;
  out->is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(*out);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(out->width);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*out, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*out, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*out, "z");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> iter_r(*out, "r");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> iter_g(*out, "g");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> iter_b(*out, "b");

  for (std::uint32_t i = 0; i < out->width; ++i) {
    const auto& point = cloud.points[i];
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    *iter_r = point.r;
    *iter_g = point.g;
    *iter_b = point.b;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
  }
  return true;
}

bool MakeImuMessage(const orbvi_sdk::FrameView& frame, sensor_msgs::Imu* out) {
  if (out == nullptr || frame.payload_data == nullptr ||
      frame.payload_size < kImuPayloadDoubleCount * sizeof(double)) {
    return false;
  }
  std::size_t offset = 0;
  out->header = MakeHeader(frame);
  return ReadPod(frame.payload_data, frame.payload_size, &offset, &out->orientation.x) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->orientation.y) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->orientation.z) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->orientation.w) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->angular_velocity.x) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->angular_velocity.y) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->angular_velocity.z) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->linear_acceleration.x) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->linear_acceleration.y) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->linear_acceleration.z) &&
         ReadPodArray(frame.payload_data, frame.payload_size, &offset, &out->orientation_covariance) &&
         ReadPodArray(frame.payload_data, frame.payload_size, &offset, &out->angular_velocity_covariance) &&
         ReadPodArray(frame.payload_data, frame.payload_size, &offset, &out->linear_acceleration_covariance);
}

std::string VioTopicSuffix(const orbvi_sdk::FrameView& frame) {
  const MetadataMap empty_metadata;
  const MetadataMap& metadata = frame.metadata != nullptr ? *frame.metadata : empty_metadata;
  const std::string source = LowerCopy(Trim(MetadataValue(metadata, "vio_source")));
  if (source == "optimized") {
    return "vio/odometry";
  }
  if (source == "propagated") {
    return "vio/imu_prediction";
  }
  return "";
}

bool MakeLivoxCustomMessage(
    const orbvi_sdk::FrameView& frame,
    livox_ros_driver2::CustomMsg* out) {
  if (out == nullptr || frame.payload_data == nullptr ||
      frame.payload_size < kLivoxCustomHeaderBytes) {
    return false;
  }
  std::size_t offset = 0;
  out->header = MakeHeader(frame);
  if (!ReadPod(frame.payload_data, frame.payload_size, &offset, &out->timebase) ||
      !ReadPod(frame.payload_data, frame.payload_size, &offset, &out->point_num) ||
      !ReadPod(frame.payload_data, frame.payload_size, &offset, &out->lidar_id)) {
    return false;
  }
  for (std::size_t i = 0; i < out->rsvd.size(); ++i) {
    if (!ReadPod(frame.payload_data, frame.payload_size, &offset, &out->rsvd[i])) {
      return false;
    }
  }
  if (out->point_num > (frame.payload_size - offset) / kLivoxCustomPointBytes) {
    return false;
  }
  out->points.resize(out->point_num);
  for (std::uint32_t i = 0; i < out->point_num; ++i) {
    auto& point = out->points[i];
    if (!ReadPod(frame.payload_data, frame.payload_size, &offset, &point.offset_time) ||
        !ReadPod(frame.payload_data, frame.payload_size, &offset, &point.x) ||
        !ReadPod(frame.payload_data, frame.payload_size, &offset, &point.y) ||
        !ReadPod(frame.payload_data, frame.payload_size, &offset, &point.z) ||
        !ReadPod(frame.payload_data, frame.payload_size, &offset, &point.reflectivity) ||
        !ReadPod(frame.payload_data, frame.payload_size, &offset, &point.tag) ||
        !ReadPod(frame.payload_data, frame.payload_size, &offset, &point.line)) {
      return false;
    }
  }
  return true;
}

bool MakeLivoxPointCloud2(
    const livox_ros_driver2::CustomMsg& msg,
    sensor_msgs::PointCloud2* out) {
  if (out == nullptr) {
    return false;
  }

  out->header = msg.header;
  out->height = 1;
  out->width = static_cast<std::uint32_t>(
      std::min<std::size_t>(msg.points.size(), std::numeric_limits<std::uint32_t>::max()));
  out->is_bigendian = false;
  out->is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(*out);
  modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::FLOAT32);
  modifier.resize(out->width);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*out, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*out, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*out, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(*out, "intensity");

  for (std::uint32_t i = 0; i < out->width; ++i) {
    const auto& point = msg.points[i];
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    *iter_intensity = static_cast<float>(point.reflectivity);
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }
  return true;
}

bool MakeVioOdometry(const orbvi_sdk::FrameView& frame, nav_msgs::Odometry* out) {
  if (out == nullptr || frame.payload_data == nullptr ||
      frame.payload_size < kVioPayloadDoubleCount * sizeof(double)) {
    return false;
  }
  const MetadataMap empty_metadata;
  const MetadataMap& metadata = frame.metadata != nullptr ? *frame.metadata : empty_metadata;
  std::size_t offset = 0;
  out->header = MakeHeader(
      frame.timestamp_ns,
      MetadataValue(metadata, "source_frame_id", frame.frame_id));
  out->child_frame_id = MetadataValue(metadata, "source_child_frame_id", "body");
  return ReadPod(frame.payload_data, frame.payload_size, &offset, &out->pose.pose.position.x) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->pose.pose.position.y) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->pose.pose.position.z) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->pose.pose.orientation.x) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->pose.pose.orientation.y) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->pose.pose.orientation.z) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->pose.pose.orientation.w) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->twist.twist.linear.x) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->twist.twist.linear.y) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->twist.twist.linear.z) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->twist.twist.angular.x) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->twist.twist.angular.y) &&
         ReadPod(frame.payload_data, frame.payload_size, &offset, &out->twist.twist.angular.z) &&
         ReadPodArray(frame.payload_data, frame.payload_size, &offset, &out->pose.covariance) &&
         ReadPodArray(frame.payload_data, frame.payload_size, &offset, &out->twist.covariance);
}

bool AppendVioOdometryToPath(
    const nav_msgs::Odometry& odometry,
    std::size_t max_poses,
    nav_msgs::Path* path) {
  if (path == nullptr || max_poses == 0u || odometry.header.frame_id.empty()) {
    return false;
  }

  const bool frame_changed =
      !path->header.frame_id.empty() &&
      path->header.frame_id != odometry.header.frame_id;
  const bool timestamp_rolled_back =
      !path->poses.empty() &&
      odometry.header.stamp < path->poses.back().header.stamp;
  if (frame_changed || timestamp_rolled_back) {
    path->poses.clear();
  }

  geometry_msgs::PoseStamped pose;
  pose.header = odometry.header;
  pose.pose = odometry.pose.pose;
  path->header = odometry.header;
  path->poses.push_back(std::move(pose));
  if (path->poses.size() > max_poses) {
    path->poses.erase(
        path->poses.begin(),
        path->poses.begin() +
            static_cast<std::ptrdiff_t>(path->poses.size() - max_poses));
  }
  return true;
}

}  // namespace orbvi_ros_bridge
