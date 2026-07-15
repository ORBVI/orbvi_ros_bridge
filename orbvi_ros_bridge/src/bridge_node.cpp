#include "bridge_node.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "frame_conversions.hpp"
#include "panorama_options.hpp"

namespace orbvi_ros_bridge {
namespace {

struct BridgeConfig {
  std::string host = "127.0.0.1";
  std::uint16_t control_port = 18088;
  std::string topic_prefix = "/orbvi";
  std::vector<orbvi_sdk::StreamId> streams = {
      orbvi_sdk::StreamId::RawFisheye,
      orbvi_sdk::StreamId::RectifiedFisheye,
      orbvi_sdk::StreamId::Imu,
      orbvi_sdk::StreamId::LidarPointCloud,
      orbvi_sdk::StreamId::LidarImu,
      orbvi_sdk::StreamId::Disparity,
      orbvi_sdk::StreamId::VioPose,
  };
  bool require_streaming_transport = true;
  bool allow_sample_endpoint_fallback = false;
  bool split_multi_image_frames = true;
  bool publish_compressed_images = true;
  bool publish_decoded_images = true;
  bool publish_depth = false;
  bool publish_depth_viz = true;
  bool publish_depth_pointcloud = true;
  bool publish_depth_panorama = true;
  bool colorize_depth_pointcloud = true;
  bool publish_panorama = false;
  bool publish_lidar_pcl = false;
  orbvi_sdk::DepthGenerationOptions depth_options;
  DepthVisualizationOptions depth_visualization_options;
  orbvi_sdk::PointCloudGenerationOptions pointcloud_options;
  orbvi_sdk::PointCloudColorizationOptions pointcloud_color_options;
  orbvi_sdk::PanoramaSubscribeOptions panorama_options;
  std::uint32_t connect_timeout_ms = 2000;
  std::uint32_t connect_retry_count = 4;
  std::uint32_t connect_retry_delay_ms = 1000;
  std::uint32_t first_frame_timeout_ms = 5000;
  std::size_t max_receive_queue_depth = 8;
  std::size_t max_decode_queue_depth = 8;
  std::uint32_t max_decode_latency_ms = 100;
  std::uint32_t queue_size = 4;
  orbvi_sdk::DecodeMode decode_mode = orbvi_sdk::DecodeMode::RawAndDecoded;
};

BridgeConfig DefaultConfig() {
  return BridgeConfig();
}

std::string NormalizeToken(std::string value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  value = value.substr(begin, end - begin + 1);
  std::transform(
      value.begin(),
      value.end(),
      value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  std::replace(value.begin(), value.end(), '-', '_');
  return value;
}

bool ParseBoolToken(const std::string& value, bool* out) {
  if (out == nullptr) {
    return false;
  }
  const std::string token = NormalizeToken(value);
  if (token == "true" || token == "1" || token == "yes" || token == "on") {
    *out = true;
    return true;
  }
  if (token == "false" || token == "0" || token == "no" || token == "off") {
    *out = false;
    return true;
  }
  return false;
}

bool ParseDepthPixelFormat(const std::string& value, orbvi_sdk::DepthPixelFormat* out) {
  if (out == nullptr) {
    return false;
  }
  const std::string token = NormalizeToken(value);
  if (token == "float32" || token == "32fc1" || token == "meters" ||
      token == "float32_meters") {
    *out = orbvi_sdk::DepthPixelFormat::Float32Meters;
    return true;
  }
  if (token == "uint16" || token == "16uc1" || token == "millimeters" ||
      token == "uint16_millimeters" || token == "uint16_mm") {
    *out = orbvi_sdk::DepthPixelFormat::Uint16Millimeters;
    return true;
  }
  return false;
}

bool ParseDepthInvalidPolicy(const std::string& value, orbvi_sdk::DepthInvalidPolicy* out) {
  if (out == nullptr) {
    return false;
  }
  const std::string token = NormalizeToken(value);
  if (token == "nan") {
    *out = orbvi_sdk::DepthInvalidPolicy::NaN;
    return true;
  }
  if (token == "zero") {
    *out = orbvi_sdk::DepthInvalidPolicy::Zero;
    return true;
  }
  if (token == "max_range" || token == "maxrange") {
    *out = orbvi_sdk::DepthInvalidPolicy::MaxRange;
    return true;
  }
  return false;
}

bool ParsePointCloudFrame(const std::string& value, orbvi_sdk::PointCloudFrame* out) {
  if (out == nullptr) {
    return false;
  }
  const std::string token = NormalizeToken(value);
  if (token == "ros" || token == "ros_body" || token == "base_link") {
    *out = orbvi_sdk::PointCloudFrame::RosBody;
    return true;
  }
  if (token == "world" || token == "calibration_world") {
    *out = orbvi_sdk::PointCloudFrame::CalibrationWorld;
    return true;
  }
  if (token == "tile" || token == "tile_optical" || token == "optical") {
    *out = orbvi_sdk::PointCloudFrame::TileOptical;
    return true;
  }
  return false;
}

bool ApplyPanoramaProfile(const std::string& value, orbvi_sdk::PanoramaStitchOptions* options) {
  if (options == nullptr) {
    return false;
  }
  const std::string token = NormalizeToken(value);
  if (token.empty() || token == "baseline" || token == "quality") {
    return true;
  }
  if (token == "ghost_suppression" || token == "ghost" || token == "balanced") {
    options->blend_mode = orbvi_sdk::PanoramaBlendMode::MultiBand;
    options->seam_blend_px = 32;
    options->photometric_align = true;
    options->seam_mode = orbvi_sdk::PanoramaSeamMode::DynamicProgramming;
    options->seam_ghost_suppression = true;
    options->seam_ghost_threshold = 80.0;
    return true;
  }
  if (token == "primary_only" || token == "primary" || token == "hard" || token == "fast") {
    options->blend_mode = orbvi_sdk::PanoramaBlendMode::PrimaryOnly;
    options->seam_blend_px = 0;
    options->photometric_align = false;
    options->seam_ghost_suppression = false;
    return true;
  }
  return false;
}

bool HasStream(const std::vector<orbvi_sdk::StreamId>& streams, orbvi_sdk::StreamId stream) {
  return std::find(streams.begin(), streams.end(), stream) != streams.end();
}

std::uint64_t TimestampDeltaNs(std::uint64_t first, std::uint64_t second) {
  return first >= second ? first - second : second - first;
}

bool RectifiedLeftPairIdFromFrameId(const std::string& frame_id, std::uint32_t* pair_id) {
  if (pair_id == nullptr) {
    return false;
  }
  if (frame_id == "orbvi/rectified/front/left") {
    *pair_id = 0;
    return true;
  }
  if (frame_id == "orbvi/rectified/right/left") {
    *pair_id = 1;
    return true;
  }
  if (frame_id == "orbvi/rectified/rear/left") {
    *pair_id = 2;
    return true;
  }
  if (frame_id == "orbvi/rectified/left/left") {
    *pair_id = 3;
    return true;
  }
  return false;
}

orbvi_sdk::OwnedFrame CopyFrameView(const orbvi_sdk::FrameView& frame) {
  orbvi_sdk::OwnedFrame out;
  out.header.stream_id = frame.stream_id;
  out.header.format = frame.format;
  out.header.compression = frame.compression;
  out.header.timestamp_ns = frame.timestamp_ns;
  out.frame_id = frame.frame_id;
  out.header.frame_id_size = static_cast<std::uint16_t>(out.frame_id.size());
  if (frame.metadata != nullptr) {
    out.metadata = *frame.metadata;
  }
  if (!frame.calibration_version.empty()) {
    out.metadata["calibration_version"] = frame.calibration_version;
  }
  if (!frame.calibration_hash.empty()) {
    out.metadata["calibration_hash"] = frame.calibration_hash;
  }
  if (frame.payload_data != nullptr && frame.payload_size > 0) {
    out.payload.assign(frame.payload_data, frame.payload_data + frame.payload_size);
  }
  out.header.payload_size = static_cast<std::uint32_t>(out.payload.size());
  return out;
}

}  // namespace

class BridgeNode::Impl {
 public:
  Impl(ros::NodeHandle node, ros::NodeHandle private_node)
      : node_(std::move(node)), private_node_(std::move(private_node)) {}

  ~Impl() {
    Stop();
  }

  bool Start() {
    config_ = LoadConfig();
    {
      std::lock_guard<std::mutex> lock(panorama_depth_mutex_);
      panorama_depth_frames_.clear();
    }
    orbvi_sdk::ClientOptions options;
    options.host = config_.host;
    options.control_port = config_.control_port;
    options.connect_timeout_ms = config_.connect_timeout_ms;
    options.max_receive_queue_depth = config_.max_receive_queue_depth;
    options.allow_sample_endpoint_fallback = config_.allow_sample_endpoint_fallback;
    options.default_decode_policy.mode = config_.decode_mode;
    options.default_decode_policy.max_decode_queue_depth = config_.max_decode_queue_depth;
    options.default_decode_policy.max_decode_latency_ms = config_.max_decode_latency_ms;

    client_.reset(new orbvi_sdk::Client(options));
    if (!ConnectWithRetry()) {
      client_.reset();
      return false;
    }

    PrepareDepthPublisher();
    StartDepthDerivedWorker();

    for (const auto stream : config_.streams) {
      if (!Subscribe(stream)) {
        Stop();
        return false;
      }
    }
    if (config_.publish_panorama && !SubscribePanorama()) {
      Stop();
      return false;
    }

    ROS_INFO_STREAM(
        "orbvi_ros_bridge started with Host SDK " << ORBVI_ROS_BRIDGE_HOST_SDK_LINKAGE
        << " library, host=" << config_.host << ":" << config_.control_port);
    return true;
  }

  void Stop() {
    if (panorama_subscription_) {
      panorama_subscription_->stop();
      panorama_subscription_.reset();
    }
    {
      std::lock_guard<std::mutex> lock(panorama_depth_mutex_);
      panorama_depth_frames_.clear();
    }
    for (auto& handle : subscriptions_) {
      handle.stop();
    }
    subscriptions_.clear();
    if (client_) {
      client_->disconnect();
      client_.reset();
    }
    StopDepthDerivedWorker();
    std::lock_guard<std::mutex> lock(publishers_mutex_);
    publishers_.clear();
  }

 private:
  bool ConnectWithRetry() {
    const std::uint32_t attempts = config_.connect_retry_count + 1;
    for (std::uint32_t attempt = 1; attempt <= attempts; ++attempt) {
      const auto connected = client_->connect();
      if (connected) {
        if (attempt > 1) {
          ROS_INFO_STREAM(
              "ORBVI Host SDK connect succeeded after " << attempt << " attempts");
        }
        return true;
      }

      if (attempt == attempts) {
        ROS_ERROR_STREAM(
            "ORBVI Host SDK connect failed after " << attempts
            << " attempts: " << connected.error().message);
        return false;
      }

      ROS_WARN_STREAM(
          "ORBVI Host SDK connect attempt " << attempt << "/" << attempts
          << " failed: " << connected.error().message
          << "; retrying in " << config_.connect_retry_delay_ms << " ms");
      std::this_thread::sleep_for(std::chrono::milliseconds(config_.connect_retry_delay_ms));
    }
    return false;
  }

  BridgeConfig LoadConfig() {
    BridgeConfig config = DefaultConfig();
    private_node_.param<std::string>("host", config.host, config.host);

    int port = config.control_port;
    private_node_.param<int>("control_port", port, port);
    config.control_port =
        port < 1 || port > 65535 ? 18088 : static_cast<std::uint16_t>(port);

    private_node_.param<std::string>("topic_prefix", config.topic_prefix, config.topic_prefix);
    config.topic_prefix = NormalizeTopicPrefix(config.topic_prefix);
    private_node_.param<bool>(
        "require_streaming_transport",
        config.require_streaming_transport,
        config.require_streaming_transport);
    private_node_.param<bool>(
        "allow_sample_endpoint_fallback",
        config.allow_sample_endpoint_fallback,
        config.allow_sample_endpoint_fallback);
    bool split_multi_image_frames = config.split_multi_image_frames;
    if (private_node_.getParam("publish_bundle_entries", split_multi_image_frames)) {
      config.split_multi_image_frames = split_multi_image_frames;
    }
    if (private_node_.getParam("split_multi_image_frames", split_multi_image_frames)) {
      config.split_multi_image_frames = split_multi_image_frames;
    }
    private_node_.param<bool>(
        "publish_compressed_images",
        config.publish_compressed_images,
        config.publish_compressed_images);

    int queue_size = static_cast<int>(config.queue_size);
    private_node_.param<int>("queue_size", queue_size, queue_size);
    config.queue_size = queue_size <= 0 ? 1u : static_cast<std::uint32_t>(queue_size);

    int receive_depth = static_cast<int>(config.max_receive_queue_depth);
    private_node_.param<int>("max_receive_queue_depth", receive_depth, receive_depth);
    config.max_receive_queue_depth =
        receive_depth <= 0 ? 1u : static_cast<std::size_t>(receive_depth);

    int decode_depth = static_cast<int>(config.max_decode_queue_depth);
    private_node_.param<int>("max_decode_queue_depth", decode_depth, decode_depth);
    config.max_decode_queue_depth =
        decode_depth <= 0 ? 1u : static_cast<std::size_t>(decode_depth);

    int decode_latency = static_cast<int>(config.max_decode_latency_ms);
    private_node_.param<int>("max_decode_latency_ms", decode_latency, decode_latency);
    config.max_decode_latency_ms =
        decode_latency <= 0 ? 100u : static_cast<std::uint32_t>(decode_latency);

    int connect_timeout = static_cast<int>(config.connect_timeout_ms);
    private_node_.param<int>("connect_timeout_ms", connect_timeout, connect_timeout);
    config.connect_timeout_ms =
        connect_timeout <= 0 ? 2000u : static_cast<std::uint32_t>(connect_timeout);

    int connect_retry_count = static_cast<int>(config.connect_retry_count);
    private_node_.param<int>("connect_retry_count", connect_retry_count, connect_retry_count);
    config.connect_retry_count =
        connect_retry_count < 0 ? 0u : static_cast<std::uint32_t>(connect_retry_count);

    int connect_retry_delay = static_cast<int>(config.connect_retry_delay_ms);
    private_node_.param<int>(
        "connect_retry_delay_ms",
        connect_retry_delay,
        connect_retry_delay);
    config.connect_retry_delay_ms =
        connect_retry_delay < 0 ? 0u : static_cast<std::uint32_t>(connect_retry_delay);

    int first_frame_timeout = static_cast<int>(config.first_frame_timeout_ms);
    private_node_.param<int>("first_frame_timeout_ms", first_frame_timeout, first_frame_timeout);
    config.first_frame_timeout_ms =
        first_frame_timeout <= 0 ? 5000u : static_cast<std::uint32_t>(first_frame_timeout);

    LoadImageMode(&config);
    LoadStreams(&config);
    LoadDepthOptions(&config);
    LoadPanoramaOptions(&config);
    bool publish_depth_bool = false;
    if (private_node_.getParam("publish_depth", publish_depth_bool)) {
      config.publish_depth = publish_depth_bool;
    }
    std::string publish_depth = "auto";
    if (private_node_.getParam("publish_depth", publish_depth)) {
      bool parsed = false;
      if (ParseBoolToken(publish_depth, &parsed)) {
        config.publish_depth = parsed;
      } else if (NormalizeToken(publish_depth) != "auto") {
        ROS_WARN_STREAM(
            "Unsupported publish_depth '" << publish_depth
            << "'; using stream-derived depth output setting");
      }
    }
    if (config.publish_depth && !HasStream(config.streams, orbvi_sdk::StreamId::Disparity)) {
      config.streams.push_back(orbvi_sdk::StreamId::Disparity);
      ROS_INFO("Depth output requires disparity frames; subscribing disparity_stream");
    }
    private_node_.param<bool>(
        "publish_lidar_pcl",
        config.publish_lidar_pcl,
        config.publish_lidar_pcl);
    if (config.publish_panorama && config.panorama_options.stitch.depth_assist &&
        !HasStream(config.streams, orbvi_sdk::StreamId::Disparity)) {
      config.streams.push_back(orbvi_sdk::StreamId::Disparity);
      ROS_INFO("Panorama depth assist requires disparity frames; subscribing disparity_stream");
    }
    return config;
  }

  void LoadImageMode(BridgeConfig* config) {
    std::string image_mode = "raw-and-decoded";
    private_node_.param<std::string>("image_mode", image_mode, image_mode);
    std::transform(image_mode.begin(), image_mode.end(), image_mode.begin(), ::tolower);
    std::replace(image_mode.begin(), image_mode.end(), '_', '-');
    if (image_mode == "decoded" || image_mode == "decoded-only") {
      config->decode_mode = orbvi_sdk::DecodeMode::DecodedOnly;
      config->publish_decoded_images = true;
      config->publish_compressed_images = false;
    } else if (
        image_mode == "raw-and-decoded" || image_mode == "raw+decoded" ||
        image_mode == "both") {
      config->decode_mode = orbvi_sdk::DecodeMode::RawAndDecoded;
      config->publish_decoded_images = true;
      config->publish_compressed_images = true;
    } else if (image_mode == "raw" || image_mode == "raw-only") {
      config->decode_mode = orbvi_sdk::DecodeMode::RawOnly;
      config->publish_decoded_images = false;
    } else {
      ROS_WARN_STREAM(
          "Unsupported image_mode '" << image_mode
          << "'; using raw-and-decoded so decoded image topics remain available");
      config->decode_mode = orbvi_sdk::DecodeMode::RawAndDecoded;
      config->publish_decoded_images = true;
      config->publish_compressed_images = true;
    }
  }

  void LoadStreams(BridgeConfig* config) {
    std::string streams = "raw,rectified,imu,lidar,lidar_imu,disparity,depth,vio";
    private_node_.param<std::string>("streams", streams, streams);
    config->streams.clear();
    config->publish_depth = false;
    config->publish_panorama = false;
    const auto requested = SplitCsv(streams);
    if (requested.size() == 1 && requested.front() == "all") {
      config->streams = DefaultConfig().streams;
      config->publish_depth = true;
      return;
    }
    for (const auto& item : requested) {
      const std::string token = NormalizeToken(item);
      if (token == "depth" || token == "depth_map" || token == "depth_map_stream") {
        config->publish_depth = true;
        continue;
      }
      if (IsBridgePanoramaToken(token)) {
        config->publish_panorama = true;
        continue;
      }
      orbvi_sdk::StreamId stream = orbvi_sdk::StreamId::Unknown;
      if (ParseBridgeStream(item, &stream) && stream != orbvi_sdk::StreamId::Unknown) {
        config->streams.push_back(stream);
      } else {
        ROS_WARN_STREAM("Ignoring unsupported stream name: " << item);
      }
    }
    if (config->streams.empty() && !config->publish_depth && !config->publish_panorama) {
      ROS_WARN("No valid streams configured; falling back to default bridge streams");
      config->streams = DefaultConfig().streams;
    }
  }

  void LoadPanoramaOptions(BridgeConfig* config) {
    std::string pano_profile = "baseline";
    private_node_.param<std::string>("pano_profile", pano_profile, pano_profile);
    private_node_.param<std::string>("panorama_profile", pano_profile, pano_profile);
    if (!ApplyPanoramaProfile(pano_profile, &config->panorama_options.stitch)) {
      ROS_WARN_STREAM("Unsupported pano_profile '" << pano_profile << "'; using baseline");
    }

    int pano_width = static_cast<int>(config->panorama_options.stitch.width);
    private_node_.param<int>("pano_width", pano_width, pano_width);
    private_node_.param<int>("panorama_width", pano_width, pano_width);
    config->panorama_options.stitch.width =
        pano_width <= 0 ? 2048u : static_cast<std::uint32_t>(pano_width);

    int pano_height = static_cast<int>(config->panorama_options.stitch.height);
    private_node_.param<int>("pano_height", pano_height, pano_height);
    private_node_.param<int>("panorama_height", pano_height, pano_height);
    config->panorama_options.stitch.height =
        pano_height <= 0 ? 1024u : static_cast<std::uint32_t>(pano_height);

    int crop_top = static_cast<int>(config->panorama_options.stitch.crop_top);
    private_node_.param<int>("pano_crop_top", crop_top, crop_top);
    private_node_.param<int>("panorama_crop_top", crop_top, crop_top);
    config->panorama_options.stitch.crop_top =
        crop_top < 0 ? 0u : static_cast<std::uint32_t>(crop_top);

    int crop_bottom = static_cast<int>(config->panorama_options.stitch.crop_bottom);
    private_node_.param<int>("pano_crop_bottom", crop_bottom, crop_bottom);
    private_node_.param<int>("panorama_crop_bottom", crop_bottom, crop_bottom);
    config->panorama_options.stitch.crop_bottom =
        crop_bottom < 0 ? 0u : static_cast<std::uint32_t>(crop_bottom);
    if (PanoramaOutputHeight(config->panorama_options.stitch) == 0u) {
      ROS_WARN_STREAM(
          "Invalid panorama crop " << config->panorama_options.stitch.crop_top
          << "+" << config->panorama_options.stitch.crop_bottom
          << " for full height " << config->panorama_options.stitch.height
          << "; using uncropped output");
      config->panorama_options.stitch.crop_top = 0u;
      config->panorama_options.stitch.crop_bottom = 0u;
    }

    private_node_.param<double>(
        "pano_fov_half_deg",
        config->panorama_options.stitch.fov_half_deg,
        config->panorama_options.stitch.fov_half_deg);
    private_node_.param<double>(
        "panorama_fov_half_deg",
        config->panorama_options.stitch.fov_half_deg,
        config->panorama_options.stitch.fov_half_deg);

    private_node_.param<double>(
        "pano_stitching_radius_m",
        config->panorama_options.stitch.stitching_radius_m,
        config->panorama_options.stitch.stitching_radius_m);
    private_node_.param<double>(
        "panorama_stitching_radius_m",
        config->panorama_options.stitch.stitching_radius_m,
        config->panorama_options.stitch.stitching_radius_m);
    if (!std::isfinite(config->panorama_options.stitch.stitching_radius_m) ||
        config->panorama_options.stitch.stitching_radius_m < 0.0) {
      ROS_WARN("Invalid panorama stitching radius; using 2.0 m");
      config->panorama_options.stitch.stitching_radius_m = 2.0;
    }

    int seam_blend_px = static_cast<int>(config->panorama_options.stitch.seam_blend_px);
    private_node_.param<int>("pano_seam_blend_px", seam_blend_px, seam_blend_px);
    private_node_.param<int>("panorama_seam_blend_px", seam_blend_px, seam_blend_px);
    config->panorama_options.stitch.seam_blend_px =
        seam_blend_px < 0 ? 0u : static_cast<std::uint32_t>(seam_blend_px);

    std::string seam_mode = orbvi_sdk::ToString(config->panorama_options.stitch.seam_mode);
    private_node_.param<std::string>("pano_seam_mode", seam_mode, seam_mode);
    private_node_.param<std::string>("panorama_seam_mode", seam_mode, seam_mode);
    if (!ParsePanoramaSeamMode(seam_mode, &config->panorama_options.stitch.seam_mode)) {
      ROS_WARN_STREAM("Unsupported pano_seam_mode '" << seam_mode << "'; using fixed");
      config->panorama_options.stitch.seam_mode = orbvi_sdk::PanoramaSeamMode::Fixed;
    }

    int dp_seam_band_px = static_cast<int>(config->panorama_options.stitch.dp_seam_band_px);
    private_node_.param<int>("pano_dp_seam_band_px", dp_seam_band_px, dp_seam_band_px);
    private_node_.param<int>("panorama_dp_seam_band_px", dp_seam_band_px, dp_seam_band_px);
    config->panorama_options.stitch.dp_seam_band_px =
        dp_seam_band_px < 0 ? 0u : static_cast<std::uint32_t>(dp_seam_band_px);

    private_node_.param<double>(
        "pano_dp_seam_smoothness",
        config->panorama_options.stitch.dp_seam_smoothness,
        config->panorama_options.stitch.dp_seam_smoothness);
    private_node_.param<double>(
        "panorama_dp_seam_smoothness",
        config->panorama_options.stitch.dp_seam_smoothness,
        config->panorama_options.stitch.dp_seam_smoothness);

    private_node_.param<double>(
        "pano_seam_avoidance_penalty",
        config->panorama_options.stitch.seam_avoidance_penalty,
        config->panorama_options.stitch.seam_avoidance_penalty);
    private_node_.param<double>(
        "panorama_seam_avoidance_penalty",
        config->panorama_options.stitch.seam_avoidance_penalty,
        config->panorama_options.stitch.seam_avoidance_penalty);

    std::string blend_mode = "multiband";
    private_node_.param<std::string>("pano_blend", blend_mode, blend_mode);
    private_node_.param<std::string>("panorama_blend", blend_mode, blend_mode);
    if (!ParsePanoramaBlendMode(blend_mode, &config->panorama_options.stitch.blend_mode)) {
      ROS_WARN_STREAM("Unsupported pano_blend '" << blend_mode << "'; using multiband");
      config->panorama_options.stitch.blend_mode = orbvi_sdk::PanoramaBlendMode::MultiBand;
    }

    int multiband_levels =
        static_cast<int>(config->panorama_options.stitch.multiband_levels);
    private_node_.param<int>("pano_multiband_levels", multiband_levels, multiband_levels);
    private_node_.param<int>(
        "panorama_multiband_levels", multiband_levels, multiband_levels);
    config->panorama_options.stitch.multiband_levels =
        static_cast<std::uint32_t>(std::clamp(multiband_levels, 0, 8));

    private_node_.param<bool>(
        "pano_photometric_align",
        config->panorama_options.stitch.photometric_align,
        config->panorama_options.stitch.photometric_align);
    private_node_.param<bool>(
        "panorama_photometric_align",
        config->panorama_options.stitch.photometric_align,
        config->panorama_options.stitch.photometric_align);

    private_node_.param<bool>(
        "pano_seam_ghost_suppression",
        config->panorama_options.stitch.seam_ghost_suppression,
        config->panorama_options.stitch.seam_ghost_suppression);
    private_node_.param<bool>(
        "panorama_seam_ghost_suppression",
        config->panorama_options.stitch.seam_ghost_suppression,
        config->panorama_options.stitch.seam_ghost_suppression);
    private_node_.param<double>(
        "pano_seam_ghost_threshold",
        config->panorama_options.stitch.seam_ghost_threshold,
        config->panorama_options.stitch.seam_ghost_threshold);
    private_node_.param<double>(
        "panorama_seam_ghost_threshold",
        config->panorama_options.stitch.seam_ghost_threshold,
        config->panorama_options.stitch.seam_ghost_threshold);

    private_node_.param<bool>(
        "pano_depth_assist",
        config->panorama_options.stitch.depth_assist,
        config->panorama_options.stitch.depth_assist);
    private_node_.param<bool>(
        "panorama_depth_assist",
        config->panorama_options.stitch.depth_assist,
        config->panorama_options.stitch.depth_assist);
    private_node_.param<bool>(
        "pano_depth_required",
        config->panorama_options.stitch.depth_required,
        config->panorama_options.stitch.depth_required);
    private_node_.param<bool>(
        "panorama_depth_required",
        config->panorama_options.stitch.depth_required,
        config->panorama_options.stitch.depth_required);
    if (config->panorama_options.stitch.depth_required &&
        !config->panorama_options.stitch.depth_assist) {
      ROS_WARN("pano_depth_required=true implies pano_depth_assist=true");
      config->panorama_options.stitch.depth_assist = true;
    }

    private_node_.param<double>(
        "pano_depth_min_range_m",
        config->panorama_options.stitch.depth_min_range_m,
        config->panorama_options.stitch.depth_min_range_m);
    private_node_.param<double>(
        "panorama_depth_min_range_m",
        config->panorama_options.stitch.depth_min_range_m,
        config->panorama_options.stitch.depth_min_range_m);
    if (config->panorama_options.stitch.depth_min_range_m <= 0.0) {
      config->panorama_options.stitch.depth_min_range_m = 0.2;
    }
    private_node_.param<double>(
        "pano_depth_max_warp_range_m",
        config->panorama_options.stitch.depth_max_warp_range_m,
        config->panorama_options.stitch.depth_max_warp_range_m);
    private_node_.param<double>(
        "panorama_depth_max_warp_range_m",
        config->panorama_options.stitch.depth_max_warp_range_m,
        config->panorama_options.stitch.depth_max_warp_range_m);
    if (config->panorama_options.stitch.depth_max_warp_range_m <=
        config->panorama_options.stitch.depth_min_range_m) {
      ROS_WARN("Invalid panorama depth warp range; using max range 8.0 m");
      config->panorama_options.stitch.depth_max_warp_range_m = 8.0;
    }

    int max_depth_delta_ms = static_cast<int>(
        config->panorama_options.max_depth_timestamp_delta_ns / 1'000'000ULL);
    private_node_.param<int>(
        "pano_max_depth_timestamp_delta_ms", max_depth_delta_ms, max_depth_delta_ms);
    private_node_.param<int>(
        "panorama_max_depth_timestamp_delta_ms", max_depth_delta_ms, max_depth_delta_ms);
    config->panorama_options.max_depth_timestamp_delta_ns =
        static_cast<std::uint64_t>(std::max(0, max_depth_delta_ms)) * 1'000'000ULL;

    int max_stitch_threads =
        static_cast<int>(config->panorama_options.stitch.max_stitch_threads);
    private_node_.param<int>(
        "pano_max_stitch_threads", max_stitch_threads, max_stitch_threads);
    private_node_.param<int>(
        "panorama_max_stitch_threads", max_stitch_threads, max_stitch_threads);
    config->panorama_options.stitch.max_stitch_threads =
        static_cast<std::uint32_t>(std::clamp(max_stitch_threads, 0, 8));

    if (config->panorama_options.stitch.depth_assist &&
        !SupportsDepthAssistedSeamRoi(config->panorama_options.stitch)) {
      ROS_WARN(
          "Panorama depth assist needs fixed-seam MultiBand, seam_blend_px>0 and "
          "seam_ghost_suppression=false; current frames will use rotation-only fallback");
    }
  }

  void LoadDepthOptions(BridgeConfig* config) {
    std::string depth_format = "float32";
    private_node_.param<std::string>("depth_format", depth_format, depth_format);
    if (!ParseDepthPixelFormat(depth_format, &config->depth_options.output_format)) {
      ROS_WARN_STREAM(
          "Unsupported depth_format '" << depth_format << "'; using float32 meters");
      config->depth_options.output_format = orbvi_sdk::DepthPixelFormat::Float32Meters;
    }

    std::string invalid_policy = "nan";
    private_node_.param<std::string>(
        "depth_invalid_policy",
        invalid_policy,
        invalid_policy);
    if (!ParseDepthInvalidPolicy(invalid_policy, &config->depth_options.invalid_policy)) {
      ROS_WARN_STREAM(
          "Unsupported depth_invalid_policy '" << invalid_policy << "'; using NaN");
      config->depth_options.invalid_policy = orbvi_sdk::DepthInvalidPolicy::NaN;
    }

    private_node_.param<double>(
        "depth_min_disparity_px",
        config->depth_options.min_disparity_px,
        config->depth_options.min_disparity_px);
    if (config->depth_options.min_disparity_px <= 0.0) {
      config->depth_options.min_disparity_px = 0.01;
    }

    private_node_.param<double>(
        "depth_max_depth_m",
        config->depth_options.max_depth_m,
        config->depth_options.max_depth_m);
    if (config->depth_options.max_depth_m <= 0.0) {
      config->depth_options.max_depth_m = 50.0;
    }

    private_node_.param<bool>(
        "publish_depth_viz",
        config->publish_depth_viz,
        config->publish_depth_viz);
    private_node_.param<bool>(
        "publish_depth_pointcloud",
        config->publish_depth_pointcloud,
        config->publish_depth_pointcloud);
    private_node_.param<bool>(
        "colorize_depth_pointcloud",
        config->colorize_depth_pointcloud,
        config->colorize_depth_pointcloud);
    private_node_.param<bool>(
        "depth_pointcloud_colorize",
        config->colorize_depth_pointcloud,
        config->colorize_depth_pointcloud);
    double color_max_delta_ms =
        static_cast<double>(config->pointcloud_color_options.max_timestamp_delta_ns) / 1.0e6;
    private_node_.param<double>(
        "depth_pointcloud_color_max_delta_ms",
        color_max_delta_ms,
        color_max_delta_ms);
    if (color_max_delta_ms >= 0.0) {
      config->pointcloud_color_options.max_timestamp_delta_ns =
          static_cast<std::uint64_t>(color_max_delta_ms * 1.0e6);
    }
    private_node_.param<double>(
        "depth_viz_min_depth_m",
        config->depth_visualization_options.min_depth_m,
        config->depth_visualization_options.min_depth_m);
    private_node_.param<double>(
        "depth_viz_max_depth_m",
        config->depth_visualization_options.max_depth_m,
        config->depth_visualization_options.max_depth_m);
    if (config->depth_visualization_options.max_depth_m <=
        config->depth_visualization_options.min_depth_m) {
      config->depth_visualization_options.min_depth_m = 0.1;
      config->depth_visualization_options.max_depth_m = 20.0;
    }
    int pointcloud_stride =
        static_cast<int>(config->depth_visualization_options.pointcloud_stride);
    private_node_.param<int>("depth_pointcloud_stride", pointcloud_stride, pointcloud_stride);
    config->depth_visualization_options.pointcloud_stride =
        pointcloud_stride <= 0 ? 1u : static_cast<std::uint32_t>(pointcloud_stride);
    config->pointcloud_options.sample_stride =
        config->depth_visualization_options.pointcloud_stride;
    config->pointcloud_options.min_depth_m = config->depth_visualization_options.min_depth_m;
    config->pointcloud_options.max_depth_m = config->depth_visualization_options.max_depth_m;
    config->pointcloud_options.min_disparity_px =
        std::max(0.5, config->depth_options.min_disparity_px);
    private_node_.param<double>(
        "depth_pointcloud_max_disp_jump_px",
        config->pointcloud_options.max_disparity_jump_px,
        config->pointcloud_options.max_disparity_jump_px);
    std::string pointcloud_frame = "ros";
    private_node_.param<std::string>(
        "depth_pointcloud_frame",
        pointcloud_frame,
        pointcloud_frame);
    if (!ParsePointCloudFrame(pointcloud_frame, &config->pointcloud_options.output_frame)) {
      ROS_WARN_STREAM(
          "Unsupported depth_pointcloud_frame '" << pointcloud_frame << "'; using ros");
      config->pointcloud_options.output_frame = orbvi_sdk::PointCloudFrame::RosBody;
    }
    private_node_.param<std::string>(
        "depth_pointcloud_frame_id",
        config->pointcloud_options.frame_id,
        config->pointcloud_options.frame_id);
  }

  void PrepareDepthPublisher() {
    depth_calibration_.reset();
    depth_panorama_lookup_.reset();
    if (!config_.publish_depth && !NeedsPanoramaDepth()) {
      return;
    }
    auto calibration_status = client_->getCalibration("");
    if (!calibration_status) {
      ROS_ERROR_STREAM(
          "Depth output disabled: failed to fetch ORBVI calibration: "
          << calibration_status.error().message);
      config_.publish_depth = false;
      config_.panorama_options.stitch.depth_assist = false;
      return;
    }
    auto calibration = orbvi_sdk::MakeDepthCalibration(calibration_status.value());
    if (!calibration) {
      ROS_ERROR_STREAM(
          "Depth output disabled: invalid ORBVI depth calibration: "
          << calibration.error().message);
      config_.publish_depth = false;
      config_.panorama_options.stitch.depth_assist = false;
      return;
    }
    depth_calibration_ = calibration.value();
    if (NeedsPanoramaDepth()) {
      auto panorama_calibration =
          orbvi_sdk::MakePanoramaCalibration(calibration_status.value());
      if (!panorama_calibration) {
        ROS_ERROR_STREAM(
            "Panorama depth assistance disabled: invalid fisheye extrinsics: "
            << panorama_calibration.error().message);
        config_.panorama_options.stitch.depth_assist = false;
      } else {
        panorama_depth_origin_reference_m_ =
            panorama_calibration.value().panorama_origin_reference_m;
        for (const auto& camera : panorama_calibration.value().cameras) {
          ROS_INFO_STREAM(
              "Panorama fisheye extrinsic role=" << camera.intrinsics.role
              << " camera_id=" << camera.intrinsics.camera_id
              << " translation_m=[" << camera.translation_cam_in_reference_m[0]
              << "," << camera.translation_cam_in_reference_m[1]
              << "," << camera.translation_cam_in_reference_m[2] << "]"
              << " rotation=[" << camera.rotation_cam_to_reference[0]
              << "," << camera.rotation_cam_to_reference[1]
              << "," << camera.rotation_cam_to_reference[2]
              << "," << camera.rotation_cam_to_reference[3]
              << "," << camera.rotation_cam_to_reference[4]
              << "," << camera.rotation_cam_to_reference[5]
              << "," << camera.rotation_cam_to_reference[6]
              << "," << camera.rotation_cam_to_reference[7]
              << "," << camera.rotation_cam_to_reference[8] << "]"
              << " full_extrinsics="
              << (camera.has_full_extrinsics ? "true" : "false"));
        }
      }
    }
    if (config_.publish_depth_panorama) {
      DepthPanoramaLookup lookup;
      std::string error;
      if (BuildDepthPanoramaLookup(*depth_calibration_, &lookup, &error)) {
        depth_panorama_lookup_ = std::move(lookup);
      } else {
        config_.publish_depth_panorama = false;
        ROS_WARN_STREAM("Depth panorama output disabled: " << error);
      }
    }
    ROS_INFO_STREAM(
        "Depth output enabled on /depth, format="
        << orbvi_sdk::ToString(config_.depth_options.output_format)
        << ", invalid_policy=" << orbvi_sdk::ToString(config_.depth_options.invalid_policy)
        << ", max_depth_m=" << config_.depth_options.max_depth_m
        << ", viz=" << (config_.publish_depth_viz ? "true" : "false")
        << ", pointcloud=" << (config_.publish_depth_pointcloud ? "true" : "false")
        << ", pointcloud_stride="
        << config_.pointcloud_options.sample_stride
        << ", pointcloud_frame=" << orbvi_sdk::ToString(config_.pointcloud_options.output_frame)
        << ", pointcloud_colorize="
        << (config_.colorize_depth_pointcloud ? "true" : "false")
        << ", panorama="
        << (config_.publish_depth_panorama && depth_panorama_lookup_ ? "true" : "false")
        << ", pointcloud_color_max_delta_ms="
        << (static_cast<double>(config_.pointcloud_color_options.max_timestamp_delta_ns) / 1.0e6)
        << ", pointcloud_max_disp_jump_px="
        << config_.pointcloud_options.max_disparity_jump_px);
  }

  void StartDepthDerivedWorker() {
    StopDepthDerivedWorker();
    if ((!config_.publish_depth && !NeedsPanoramaDepth()) ||
        (!config_.publish_depth_pointcloud && !config_.publish_depth_panorama &&
         !NeedsPanoramaDepth())) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(depth_derived_mutex_);
      depth_derived_stop_ = false;
      depth_derived_pending_ = false;
    }
    depth_derived_worker_ = std::thread([this]() { DepthDerivedWorkerLoop(); });
  }

  void StopDepthDerivedWorker() {
    {
      std::lock_guard<std::mutex> lock(depth_derived_mutex_);
      depth_derived_stop_ = true;
      depth_derived_pending_ = false;
    }
    depth_derived_cv_.notify_all();
    if (depth_derived_worker_.joinable()) {
      depth_derived_worker_.join();
    }
    {
      std::lock_guard<std::mutex> lock(depth_derived_mutex_);
      depth_derived_stop_ = false;
    }
  }

  void EnqueueDepthDerivedFrame(const orbvi_sdk::FrameView& frame) {
    if ((!config_.publish_depth && !NeedsPanoramaDepth()) ||
        (!config_.publish_depth_pointcloud && !config_.publish_depth_panorama &&
         !NeedsPanoramaDepth())) {
      return;
    }
    orbvi_sdk::OwnedFrame copy = CopyFrameView(frame);
    {
      std::lock_guard<std::mutex> lock(depth_derived_mutex_);
      if (depth_derived_stop_) {
        return;
      }
      depth_derived_frame_ = std::move(copy);
      depth_derived_pending_ = true;
    }
    depth_derived_cv_.notify_one();
  }

  void DepthDerivedWorkerLoop() {
    while (true) {
      orbvi_sdk::OwnedFrame frame;
      {
        std::unique_lock<std::mutex> lock(depth_derived_mutex_);
        depth_derived_cv_.wait(lock, [this]() {
          return depth_derived_stop_ || depth_derived_pending_;
        });
        if (depth_derived_stop_) {
          return;
        }
        frame = std::move(depth_derived_frame_);
        depth_derived_pending_ = false;
      }
      const auto view = frame.view();
      GeneratePanoramaDepth(view);
      PublishDepthPointCloud(view);
      PublishDepthPanorama(view);
    }
  }

  bool NeedsPanoramaDepth() const {
    return config_.publish_panorama && config_.panorama_options.stitch.depth_assist;
  }

  void GeneratePanoramaDepth(const orbvi_sdk::FrameView& disparity) {
    if (!NeedsPanoramaDepth() || !depth_calibration_) {
      return;
    }
    orbvi_sdk::RigDepthPanoramaOptions options;
    options.width = config_.panorama_options.stitch.width;
    options.height = config_.panorama_options.stitch.height;
    options.crop_top = config_.panorama_options.stitch.crop_top;
    options.crop_bottom = config_.panorama_options.stitch.crop_bottom;
    options.min_range_m = config_.panorama_options.stitch.depth_min_range_m;
    options.max_range_m = std::max(
        config_.panorama_options.stitch.depth_max_warp_range_m, 20.0);
    options.origin_reference_m = panorama_depth_origin_reference_m_;
    auto generated = orbvi_sdk::GenerateRigDepthPanorama(
        disparity, *depth_calibration_, options);
    if (!generated) {
      ROS_WARN_THROTTLE(
          5.0,
          "Failed to generate rig-centered panorama depth: %s",
          generated.error().message.c_str());
      return;
    }
    auto depth = std::make_shared<orbvi_sdk::RigDepthPanorama>(
        std::move(generated.value()));
    const auto log_depth = depth;
    std::size_t cached_frames = 0;
    {
      std::lock_guard<std::mutex> lock(panorama_depth_mutex_);
      panorama_depth_frames_.push_back(std::move(depth));
      while (panorama_depth_frames_.size() > 4u) {
        panorama_depth_frames_.pop_front();
      }
      cached_frames = panorama_depth_frames_.size();
    }
    const auto build_ms = log_depth->metadata.find("build_ms");
    const auto valid_ratio = log_depth->metadata.find("valid_ratio");
    ROS_INFO_THROTTLE(
        2.0,
        "Panorama rig depth timestamp_ns=%llu cached_frames=%zu "
        "valid_ratio=%s build_ms=%s",
        static_cast<unsigned long long>(log_depth->timestamp_ns),
        cached_frames,
        valid_ratio == log_depth->metadata.end() ? "unknown" : valid_ratio->second.c_str(),
        build_ms == log_depth->metadata.end() ? "unknown" : build_ms->second.c_str());
  }

  bool Subscribe(orbvi_sdk::StreamId stream) {
    orbvi_sdk::SubscribeOptions options;
    options.decode_policy.mode =
        IsBridgeImageStream(stream) ? config_.decode_mode : orbvi_sdk::DecodeMode::RawOnly;
    options.decode_policy.max_decode_queue_depth = config_.max_decode_queue_depth;
    options.decode_policy.max_decode_latency_ms = config_.max_decode_latency_ms;
    options.max_receive_queue_depth = config_.max_receive_queue_depth;
    options.first_frame_timeout_ms = config_.first_frame_timeout_ms;
    options.require_streaming_transport = config_.require_streaming_transport;
    options.allow_sample_endpoint_fallback = config_.allow_sample_endpoint_fallback;

    auto result = client_->subscribe(
        stream,
        options,
        [this](const orbvi_sdk::FrameDelivery& delivery) {
          PublishDelivery(delivery);
        });
    if (!result) {
      ROS_ERROR_STREAM(
          "ORBVI Host SDK subscribe failed for " << orbvi_sdk::ToString(stream)
          << ": " << result.error().message);
      return false;
    }
    subscriptions_.push_back(std::move(result.value()));
    ROS_INFO_STREAM("Subscribed Host SDK stream " << orbvi_sdk::ToString(stream));
    return true;
  }

  bool SubscribePanorama() {
    orbvi_sdk::PanoramaSubscribeOptions options = config_.panorama_options;
    options.max_receive_queue_depth = config_.max_receive_queue_depth;
    options.first_frame_timeout_ms = config_.first_frame_timeout_ms;
    options.require_streaming_transport = config_.require_streaming_transport;
    options.allow_sample_endpoint_fallback = config_.allow_sample_endpoint_fallback;
    options.include_source_frame = false;

    auto result = client_->subscribePanorama(
        options,
        [this](std::uint64_t rgb_timestamp_ns) {
          return ClosestPanoramaDepth(rgb_timestamp_ns);
        },
        [this](const orbvi_sdk::PanoramaFrameDelivery& delivery) {
          PublishPanorama(delivery);
        });
    if (!result) {
      ROS_ERROR_STREAM(
          "ORBVI Host SDK panorama subscribe failed: " << result.error().message);
      return false;
    }
    panorama_subscription_ = std::move(result.value());
    ROS_INFO_STREAM(
        "Subscribed Host SDK derived panorama from raw_fisheye_stream, output_size="
        << options.stitch.width << "x" << PanoramaOutputHeight(options.stitch)
        << ", full_height=" << options.stitch.height
        << ", crop_top=" << options.stitch.crop_top
        << ", crop_bottom=" << options.stitch.crop_bottom
        << ", stitching_radius_m=" << options.stitch.stitching_radius_m
        << ", blend=" << orbvi_sdk::ToString(options.stitch.blend_mode)
        << ", multiband_levels=" << options.stitch.multiband_levels
        << ", seam_mode=" << orbvi_sdk::ToString(options.stitch.seam_mode)
        << ", dp_seam_band_px=" << options.stitch.dp_seam_band_px
        << ", dp_seam_smoothness=" << options.stitch.dp_seam_smoothness
        << ", photometric_align="
        << (options.stitch.photometric_align ? "true" : "false")
        << ", seam_ghost_suppression="
        << (options.stitch.seam_ghost_suppression ? "true" : "false")
        << ", seam_ghost_threshold=" << options.stitch.seam_ghost_threshold
        << ", depth_assist=" << (options.stitch.depth_assist ? "true" : "false")
        << ", depth_required=" << (options.stitch.depth_required ? "true" : "false")
        << ", depth_range_m=" << options.stitch.depth_min_range_m
        << ".." << options.stitch.depth_max_warp_range_m
        << ", max_depth_delta_ms="
        << (static_cast<double>(options.max_depth_timestamp_delta_ns) / 1.0e6)
        << ", depth_source=shared-rig-depth-cache"
        << ", max_stitch_threads=" << options.stitch.max_stitch_threads);
    return true;
  }

  std::shared_ptr<const orbvi_sdk::RigDepthPanorama> ClosestPanoramaDepth(
      std::uint64_t rgb_timestamp_ns) {
    std::lock_guard<std::mutex> lock(panorama_depth_mutex_);
    std::shared_ptr<const orbvi_sdk::RigDepthPanorama> best;
    std::uint64_t best_delta = std::numeric_limits<std::uint64_t>::max();
    for (const auto& depth : panorama_depth_frames_) {
      const std::uint64_t delta =
          TimestampDeltaNs(rgb_timestamp_ns, depth->timestamp_ns);
      if (delta < best_delta) {
        best = depth;
        best_delta = delta;
      }
    }
    return best;
  }

  template <typename MessageT>
  ros::Publisher Publisher(const std::string& topic) {
    std::lock_guard<std::mutex> lock(publishers_mutex_);
    auto it = publishers_.find(topic);
    if (it == publishers_.end()) {
      it = publishers_.emplace(topic, node_.advertise<MessageT>(topic, config_.queue_size)).first;
    }
    return it->second;
  }

  void PublishDelivery(const orbvi_sdk::FrameDelivery& delivery) {
    const auto frame = delivery.frame.view();
    switch (frame.stream_id) {
      case orbvi_sdk::StreamId::RawFisheye:
      case orbvi_sdk::StreamId::RectifiedFisheye:
        PublishImages(frame, delivery);
        break;
      case orbvi_sdk::StreamId::Imu:
      case orbvi_sdk::StreamId::LidarImu:
        PublishImu(frame);
        break;
      case orbvi_sdk::StreamId::LidarPointCloud:
        PublishLivox(frame);
        break;
      case orbvi_sdk::StreamId::Disparity:
        PublishDisparity(frame);
        break;
      case orbvi_sdk::StreamId::VioPose:
        PublishVio(frame);
        break;
      default:
        ROS_WARN_THROTTLE(
            5.0,
            "Ignoring unsupported Host SDK stream id %s",
            orbvi_sdk::ToString(frame.stream_id));
        break;
    }
  }

  void PublishImages(
      const orbvi_sdk::FrameView& frame,
      const orbvi_sdk::FrameDelivery& delivery) {
    CacheRectifiedLeftImages(frame, delivery);
    if (config_.publish_compressed_images) {
      std::string skipped_reason;
      for (const auto& output :
           MakeCompressedImageMessages(frame, config_.split_multi_image_frames, &skipped_reason)) {
        Publisher<sensor_msgs::CompressedImage>(
            JoinTopic(config_.topic_prefix, output.topic_suffix))
            .publish(output.message);
      }
      if (!skipped_reason.empty()) {
        ROS_WARN_THROTTLE(5.0, "%s", skipped_reason.c_str());
      }
    }
    if (config_.publish_decoded_images) {
      const auto decoded_outputs = MakeDecodedImageMessages(frame.stream_id, delivery);
      if (decoded_outputs.empty()) {
        ROS_WARN_THROTTLE(
            5.0,
            "Decoded image output was requested, but Host SDK did not deliver decoded image data");
      }
      for (const auto& output : decoded_outputs) {
        Publisher<sensor_msgs::Image>(JoinTopic(config_.topic_prefix, output.topic_suffix))
            .publish(output.message);
      }
    }
  }

  void CacheRectifiedLeftImages(
      const orbvi_sdk::FrameView& frame,
      const orbvi_sdk::FrameDelivery& delivery) {
    if (frame.stream_id != orbvi_sdk::StreamId::RectifiedFisheye ||
        !config_.colorize_depth_pointcloud) {
      return;
    }
    const auto cache_one = [this](const orbvi_sdk::OwnedDecodedImage& image) {
      std::uint32_t pair_id = 0;
      if (!RectifiedLeftPairIdFromFrameId(image.frame_id, &pair_id) ||
          image.pixel_format != orbvi_sdk::PixelFormat::Bgr888 ||
          image.data.empty() || image.width == 0 || image.height == 0 ||
          image.stride < image.width * 3u) {
        return;
      }
      std::lock_guard<std::mutex> lock(rectified_left_images_mutex_);
      auto& cache = rectified_left_images_[pair_id];
      cache.push_back(std::make_shared<orbvi_sdk::OwnedDecodedImage>(image));
      constexpr std::size_t kMaxCachedImagesPerPair = 16;
      while (cache.size() > kMaxCachedImagesPerPair) {
        cache.pop_front();
      }
    };
    for (const auto& image : delivery.decoded_images) {
      cache_one(image);
    }
    if (delivery.decoded_image) {
      cache_one(*delivery.decoded_image);
    }
  }

  std::vector<std::pair<std::uint32_t, std::shared_ptr<const orbvi_sdk::OwnedDecodedImage>>>
  SnapshotRectifiedLeftImages(std::uint64_t target_timestamp_ns) const {
    std::vector<std::pair<std::uint32_t, std::shared_ptr<const orbvi_sdk::OwnedDecodedImage>>>
        images;
    if (!depth_calibration_) {
      return images;
    }
    std::lock_guard<std::mutex> lock(rectified_left_images_mutex_);
    images.reserve(rectified_left_images_.size());
    for (const auto& entry : rectified_left_images_) {
      if (entry.first >= depth_calibration_->tile_count || entry.second.empty()) {
        continue;
      }
      auto best = entry.second.begin();
      std::uint64_t best_delta = TimestampDeltaNs((*best)->timestamp_ns, target_timestamp_ns);
      for (auto it = std::next(entry.second.begin()); it != entry.second.end(); ++it) {
        const std::uint64_t delta = TimestampDeltaNs((*it)->timestamp_ns, target_timestamp_ns);
        if (delta < best_delta) {
          best = it;
          best_delta = delta;
        }
      }
      if (best_delta <= config_.pointcloud_color_options.max_timestamp_delta_ns) {
        images.emplace_back(entry.first, *best);
      }
    }
    return images;
  }

  orbvi_sdk::Result<orbvi_sdk::PointCloud> GenerateDepthPointCloud(
      const orbvi_sdk::FrameView& source_disparity) const {
    if (config_.colorize_depth_pointcloud && depth_calibration_) {
      auto owned_images = SnapshotRectifiedLeftImages(source_disparity.timestamp_ns);
      if (owned_images.size() >= depth_calibration_->tile_count) {
        std::vector<orbvi_sdk::RectifiedLeftImageView> image_views;
        image_views.reserve(owned_images.size());
        for (const auto& entry : owned_images) {
          orbvi_sdk::RectifiedLeftImageView view;
          view.pair_id = entry.first;
          view.image = entry.second->view();
          image_views.push_back(view);
        }
        auto color_result = orbvi_sdk::GeneratePointCloud(
            source_disparity,
            *depth_calibration_,
            image_views,
            config_.pointcloud_options,
            config_.pointcloud_color_options);
        if (color_result) {
          return color_result;
        }
        ROS_WARN_THROTTLE(
            5.0,
            "Failed to generate colorized ORBVI depth PointCloud: %s; falling back",
            color_result.error().message.c_str());
      }
    }
    return orbvi_sdk::GeneratePointCloud(
        source_disparity,
        *depth_calibration_,
        config_.pointcloud_options);
  }

  void PublishPanorama(const orbvi_sdk::PanoramaFrameDelivery& delivery) {
    sensor_msgs::Image message;
    if (!MakePanoramaImageMessage(delivery.panorama.view(), &message)) {
      ROS_WARN_THROTTLE(5.0, "Failed to convert Host SDK panorama to ROS Image");
      return;
    }
    Publisher<sensor_msgs::Image>(JoinTopic(config_.topic_prefix, "pano/image"))
        .publish(message);
    const auto camera_map = delivery.panorama.metadata.find("fisheye_camera_map");
    const auto rotation =
        delivery.panorama.metadata.find("fisheye_rotation_extrinsics_applied");
    if (config_.panorama_options.stitch.depth_assist) {
      const auto applied = delivery.panorama.metadata.find("depth_assist_applied");
      const auto backend = delivery.panorama.metadata.find("multiband_backend");
      const auto translation =
          delivery.panorama.metadata.find("fisheye_translation_extrinsics_applied");
      ROS_INFO_THROTTLE(
          2.0,
          "Panorama depth assist applied=%s backend=%s cameras=%s "
          "rotation_extrinsics=%s translation_extrinsics=%s valid_ratio=%.3f "
          "rgb_depth_delta=%.2fms depth_geometry=%.2fms stitch=%.2fms "
          "assisted=%llu fallback=%llu sync_drops=%llu depth_failures=%llu",
          applied == delivery.panorama.metadata.end() ? "unknown" : applied->second.c_str(),
          backend == delivery.panorama.metadata.end() ? "unknown" : backend->second.c_str(),
          camera_map == delivery.panorama.metadata.end() ? "unknown" : camera_map->second.c_str(),
          rotation == delivery.panorama.metadata.end() ? "unknown" : rotation->second.c_str(),
          translation == delivery.panorama.metadata.end() ? "unknown" : translation->second.c_str(),
          delivery.stats.depth_valid_ratio,
          delivery.stats.rgb_depth_delta_ms,
          delivery.stats.depth_geometry_latency_ms,
          delivery.stats.stitch_latency_ms,
          static_cast<unsigned long long>(delivery.stats.depth_assisted_panoramas),
          static_cast<unsigned long long>(delivery.stats.depth_fallback_panoramas),
          static_cast<unsigned long long>(delivery.stats.depth_sync_drops),
          static_cast<unsigned long long>(delivery.stats.depth_generation_failures));
    } else {
      ROS_INFO_THROTTLE(
          2.0,
          "Panorama geometry cameras=%s rotation_extrinsics=%s "
          "translation_extrinsics=false depth_assist=false stitch=%.2fms",
          camera_map == delivery.panorama.metadata.end() ? "unknown" : camera_map->second.c_str(),
          rotation == delivery.panorama.metadata.end() ? "unknown" : rotation->second.c_str(),
          delivery.stats.stitch_latency_ms);
    }
  }

  void PublishImu(const orbvi_sdk::FrameView& frame) {
    sensor_msgs::Imu message;
    if (!MakeImuMessage(frame, &message)) {
      ROS_WARN_THROTTLE(5.0, "Failed to parse Host SDK IMU payload");
      return;
    }
    const std::string suffix =
        frame.stream_id == orbvi_sdk::StreamId::LidarImu ? "lidar/imu" : "imu";
    Publisher<sensor_msgs::Imu>(JoinTopic(config_.topic_prefix, suffix)).publish(message);
  }

  void PublishLivox(const orbvi_sdk::FrameView& frame) {
    livox_ros_driver2::CustomMsg message;
    if (!MakeLivoxCustomMessage(frame, &message)) {
      ROS_WARN_THROTTLE(5.0, "Failed to parse Host SDK Livox payload");
      return;
    }
    Publisher<livox_ros_driver2::CustomMsg>(JoinTopic(config_.topic_prefix, "lidar/custom"))
        .publish(message);
    if (config_.publish_lidar_pcl) {
      sensor_msgs::PointCloud2 pcl_message;
      if (MakeLivoxPointCloud2(message, &pcl_message)) {
        Publisher<sensor_msgs::PointCloud2>(JoinTopic(config_.topic_prefix, "lidar/points"))
            .publish(pcl_message);
      } else {
        ROS_WARN_THROTTLE(5.0, "Failed to convert Host SDK Livox payload to PointCloud2");
      }
    }
  }

  void PublishDisparity(const orbvi_sdk::FrameView& frame) {
    if (depth_calibration_) {
      const auto outputs = MakeSplitDisparityImages(frame, *depth_calibration_);
      if (!outputs.empty()) {
        for (const auto& output : outputs) {
          Publisher<sensor_msgs::Image>(JoinTopic(config_.topic_prefix, output.topic_suffix))
              .publish(output.message);
        }
      } else {
        ROS_WARN_THROTTLE(5.0, "Failed to split Host SDK disparity payload");
      }
    } else {
      sensor_msgs::Image message;
      if (!MakeDisparityImage(frame, &message)) {
        ROS_WARN_THROTTLE(5.0, "Failed to parse Host SDK disparity payload");
        return;
      }
      Publisher<sensor_msgs::Image>(JoinTopic(config_.topic_prefix, "disparity"))
          .publish(message);
    }
    if (config_.publish_depth) {
      PublishDepth(frame);
    } else {
      EnqueueDepthDerivedFrame(frame);
    }
  }

  void PublishDepth(const orbvi_sdk::FrameView& frame) {
    if (!config_.publish_depth || !depth_calibration_) {
      return;
    }
    auto depth_result =
        orbvi_sdk::GenerateDepthMap(frame, *depth_calibration_, config_.depth_options);
    if (!depth_result) {
      ROS_WARN_THROTTLE(
          5.0,
          "Failed to generate ORBVI depth image: %s",
          depth_result.error().message.c_str());
      return;
    }
    const orbvi_sdk::DepthMap& depth = depth_result.value();
    const auto outputs = MakeSplitDepthImages(depth.view(), *depth_calibration_);
    if (outputs.empty()) {
      ROS_WARN_THROTTLE(5.0, "Failed to split ORBVI depth image");
    } else {
      for (const auto& output : outputs) {
        Publisher<sensor_msgs::Image>(JoinTopic(config_.topic_prefix, output.topic_suffix))
            .publish(output.message);
      }
    }
    EnqueueDepthDerivedFrame(frame);
    PublishDepthVisualization(depth.view(), frame);
  }

  void PublishDepthPanorama(const orbvi_sdk::FrameView& frame) {
    if (!config_.publish_depth_panorama || !depth_calibration_ || !depth_panorama_lookup_) {
      return;
    }
    sensor_msgs::Image message;
    std::string error;
    if (!MakeDepthPanoramaImage(
            frame,
            *depth_calibration_,
            *depth_panorama_lookup_,
            &message,
            &error)) {
      ROS_WARN_THROTTLE(
          5.0,
          "Failed to generate ORBVI depth panorama Image: %s",
          error.c_str());
      return;
    }
    Publisher<sensor_msgs::Image>(JoinTopic(config_.topic_prefix, "depth/panorama"))
        .publish(message);
    sensor_msgs::Image viz;
    if (MakeDepthPanoramaVisualizationImage(message, &viz)) {
      Publisher<sensor_msgs::Image>(JoinTopic(config_.topic_prefix, "depth/panorama/viz"))
          .publish(viz);
    } else {
      ROS_WARN_THROTTLE(5.0, "Failed to convert ORBVI depth panorama to visualization Image");
    }
  }

  void PublishDepthPointCloud(const orbvi_sdk::FrameView& source_disparity) {
    if (!config_.publish_depth_pointcloud || !depth_calibration_) {
      return;
    }
    auto cloud_result = GenerateDepthPointCloud(source_disparity);
    if (!cloud_result) {
      ROS_WARN_THROTTLE(
          5.0,
          "Failed to generate ORBVI SDK depth PointCloud: %s",
          cloud_result.error().message.c_str());
      return;
    }
    sensor_msgs::PointCloud2 message;
    if (MakePointCloudMessage(cloud_result.value(), &message)) {
      Publisher<sensor_msgs::PointCloud2>(JoinTopic(config_.topic_prefix, "depth/points"))
          .publish(message);
    } else {
      ROS_WARN_THROTTLE(5.0, "Failed to convert ORBVI depth image to visualization PointCloud2");
    }
  }

  void PublishDepthVisualization(
      const orbvi_sdk::DepthMapView& depth,
      const orbvi_sdk::FrameView& source_disparity) {
    if (config_.publish_depth_viz) {
      const auto outputs = MakeSplitDepthVisualizationImages(
          depth,
          source_disparity,
          config_.depth_visualization_options,
          *depth_calibration_);
      if (outputs.empty()) {
        ROS_WARN_THROTTLE(5.0, "Failed to split ORBVI depth visualization Image");
        return;
      }
      for (const auto& output : outputs) {
        Publisher<sensor_msgs::Image>(JoinTopic(config_.topic_prefix, output.topic_suffix))
            .publish(output.message);
      }
    }
  }

  void PublishVio(const orbvi_sdk::FrameView& frame) {
    const std::string suffix = VioTopicSuffix(frame);
    if (suffix.empty()) {
      ROS_WARN_THROTTLE(
          5.0,
          "Dropping Host SDK VIO payload with missing or unsupported vio_source metadata");
      return;
    }
    nav_msgs::Odometry message;
    if (!MakeVioOdometry(frame, &message)) {
      ROS_WARN_THROTTLE(5.0, "Failed to parse Host SDK VIO payload");
      return;
    }
    Publisher<nav_msgs::Odometry>(JoinTopic(config_.topic_prefix, suffix)).publish(message);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  BridgeConfig config_;
  std::unique_ptr<orbvi_sdk::Client> client_;
  std::optional<orbvi_sdk::DepthCalibration> depth_calibration_;
  std::optional<DepthPanoramaLookup> depth_panorama_lookup_;
  std::optional<orbvi_sdk::PanoramaSubscriptionHandle> panorama_subscription_;
  std::vector<orbvi_sdk::SubscriptionHandle> subscriptions_;
  std::mutex panorama_depth_mutex_;
  std::deque<std::shared_ptr<const orbvi_sdk::RigDepthPanorama>> panorama_depth_frames_;
  std::array<double, 3> panorama_depth_origin_reference_m_ = {0.0, 0.0, 0.0};
  mutable std::mutex rectified_left_images_mutex_;
  std::map<
      std::uint32_t,
      std::deque<std::shared_ptr<const orbvi_sdk::OwnedDecodedImage>>>
      rectified_left_images_;
  std::mutex depth_derived_mutex_;
  std::condition_variable depth_derived_cv_;
  orbvi_sdk::OwnedFrame depth_derived_frame_;
  std::thread depth_derived_worker_;
  bool depth_derived_stop_ = false;
  bool depth_derived_pending_ = false;
  std::mutex publishers_mutex_;
  std::map<std::string, ros::Publisher> publishers_;
};

BridgeNode::BridgeNode(ros::NodeHandle node, ros::NodeHandle private_node)
    : impl_(new Impl(std::move(node), std::move(private_node))) {}

BridgeNode::~BridgeNode() = default;

bool BridgeNode::Start() {
  return impl_->Start();
}

void BridgeNode::Stop() {
  impl_->Stop();
}

}  // namespace orbvi_ros_bridge
