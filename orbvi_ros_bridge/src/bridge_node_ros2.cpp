#include "bridge_node_ros2.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "frame_conversions_ros2.hpp"

#define ROS_ERROR_STREAM(expr) RCLCPP_ERROR_STREAM(node_->get_logger(), expr)
#define ROS_WARN_STREAM(expr) RCLCPP_WARN_STREAM(node_->get_logger(), expr)
#define ROS_INFO_STREAM(expr) RCLCPP_INFO_STREAM(node_->get_logger(), expr)
#define ROS_ERROR(...) RCLCPP_ERROR(node_->get_logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(node_->get_logger(), __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(node_->get_logger(), __VA_ARGS__)
#define ROS_WARN_THROTTLE(period_sec, ...) \
  RCLCPP_WARN_THROTTLE( \
      node_->get_logger(), \
      *node_->get_clock(), \
      static_cast<int>((period_sec) * 1000.0), \
      __VA_ARGS__)
#define ROS_INFO_THROTTLE(period_sec, ...) \
  RCLCPP_INFO_THROTTLE( \
      node_->get_logger(), \
      *node_->get_clock(), \
      static_cast<int>((period_sec) * 1000.0), \
      __VA_ARGS__)

namespace orbvi_ros_bridge {
namespace {

struct BridgeConfig {
  std::string host = "127.0.0.1";
  std::uint16_t control_port = 18088;
  std::string topic_prefix = "/orbvi";
  std::vector<std::string> raw_camera_ids;
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
  bool sensor_data_best_effort = true;
  bool log_delivery_stats = false;
  orbvi_sdk::DepthGenerationOptions depth_options;
  DepthVisualizationOptions depth_visualization_options;
  orbvi_sdk::PointCloudGenerationOptions pointcloud_options;
  std::uint32_t connect_timeout_ms = 2000;
  std::uint32_t connect_retry_count = 4;
  std::uint32_t connect_retry_delay_ms = 1000;
  std::uint32_t first_frame_timeout_ms = 5000;
  std::size_t max_receive_queue_depth = 64;
  std::size_t max_decode_queue_depth = 64;
  std::size_t max_publish_queue_depth = 64;
  std::uint32_t max_decode_latency_ms = 100;
  std::uint32_t queue_size = 10;
  std::uint32_t publish_worker_threads = 8;
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

bool HasStream(const std::vector<orbvi_sdk::StreamId>& streams, orbvi_sdk::StreamId stream) {
  return std::find(streams.begin(), streams.end(), stream) != streams.end();
}

}  // namespace

class ParameterReader {
 public:
  explicit ParameterReader(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

  template <typename T>
  void param(const std::string& name, T& value, const T& default_value) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<T>(name, default_value);
    }
    getParam(name, value);
  }

  template <typename T>
  bool getParam(const std::string& name, T& value) const {
    if (!node_->has_parameter(name)) {
      return false;
    }
    try {
      value = node_->get_parameter(name).get_value<T>();
      return true;
    } catch (const rclcpp::ParameterTypeException&) {
      return false;
    }
  }

 private:
  rclcpp::Node::SharedPtr node_;
};

template <>
bool ParameterReader::getParam<std::string>(const std::string& name, std::string& value) const {
  if (!node_->has_parameter(name)) {
    return false;
  }
  const auto parameter = node_->get_parameter(name);
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
    value = parameter.as_bool() ? "true" : "false";
    return true;
  }
  try {
    value = parameter.as_string();
    return true;
  } catch (const rclcpp::ParameterTypeException&) {
    return false;
  }
}

template <>
bool ParameterReader::getParam<bool>(const std::string& name, bool& value) const {
  if (!node_->has_parameter(name)) {
    return false;
  }
  const auto parameter = node_->get_parameter(name);
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    return ParseBoolToken(parameter.as_string(), &value);
  }
  try {
    value = parameter.as_bool();
    return true;
  } catch (const rclcpp::ParameterTypeException&) {
    return false;
  }
}

class BridgeNode::Impl {
 public:
  explicit Impl(rclcpp::Node::SharedPtr node)
      : node_(std::move(node)), private_node_(node_) {}

  ~Impl() {
    Stop();
  }

  bool Start() {
    config_ = LoadConfig();
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

    StartPublishWorkers();
    PrepareDepthPublisher();

    for (const auto stream : config_.streams) {
      if (!Subscribe(stream)) {
        Stop();
        return false;
      }
    }

    ROS_INFO_STREAM(
        "orbvi_ros_bridge started with Host SDK " << ORBVI_ROS_BRIDGE_HOST_SDK_LINKAGE
        << " library, host=" << config_.host << ":" << config_.control_port);
    return true;
  }

  void Stop() {
    for (auto& handle : subscriptions_) {
      handle.stop();
    }
    subscriptions_.clear();
    if (client_) {
      client_->disconnect();
      client_.reset();
    }
    StopPublishWorkers();
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
    std::string raw_camera_ids;
    private_node_.param<std::string>("raw_camera_ids", raw_camera_ids, raw_camera_ids);
    config.raw_camera_ids = SplitCsv(raw_camera_ids);
    for (auto& raw_camera_id : config.raw_camera_ids) {
      raw_camera_id = NormalizeToken(raw_camera_id);
      const std::string prefix = "camera_";
      if (raw_camera_id.find(prefix) == 0) {
        raw_camera_id = raw_camera_id.substr(prefix.size());
      }
    }
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
    private_node_.param<bool>(
        "sensor_data_best_effort",
        config.sensor_data_best_effort,
        config.sensor_data_best_effort);
    private_node_.param<bool>(
        "log_delivery_stats",
        config.log_delivery_stats,
        config.log_delivery_stats);

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

    int publish_queue_depth = static_cast<int>(config.max_publish_queue_depth);
    private_node_.param<int>(
        "max_publish_queue_depth",
        publish_queue_depth,
        publish_queue_depth);
    config.max_publish_queue_depth =
        publish_queue_depth <= 0 ? 1u : static_cast<std::size_t>(publish_queue_depth);

    int publish_workers = static_cast<int>(config.publish_worker_threads);
    private_node_.param<int>("publish_worker_threads", publish_workers, publish_workers);
    config.publish_worker_threads =
        publish_workers < 0 ? 0u : static_cast<std::uint32_t>(publish_workers);

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
      orbvi_sdk::StreamId stream = orbvi_sdk::StreamId::Unknown;
      if (ParseBridgeStream(item, &stream) && stream != orbvi_sdk::StreamId::Unknown) {
        config->streams.push_back(stream);
      } else {
        ROS_WARN_STREAM("Ignoring unsupported stream name: " << item);
      }
    }
    if (config->streams.empty()) {
      ROS_WARN("No valid streams configured; falling back to default bridge streams");
      config->streams = DefaultConfig().streams;
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
    if (!config_.publish_depth) {
      return;
    }
    auto calibration_status = client_->getCalibration("");
    if (!calibration_status) {
      ROS_ERROR_STREAM(
          "Depth output disabled: failed to fetch ORBVI calibration: "
          << calibration_status.error().message);
      config_.publish_depth = false;
      return;
    }
    auto calibration = orbvi_sdk::MakeDepthCalibration(calibration_status.value());
    if (!calibration) {
      ROS_ERROR_STREAM(
          "Depth output disabled: invalid ORBVI depth calibration: "
          << calibration.error().message);
      config_.publish_depth = false;
      return;
    }
    depth_calibration_ = calibration.value();
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
        << ", pointcloud_max_disp_jump_px="
        << config_.pointcloud_options.max_disparity_jump_px);
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

  template <typename MessageT>
  typename rclcpp::Publisher<MessageT>::SharedPtr Publisher(const std::string& topic) {
    std::lock_guard<std::mutex> lock(publishers_mutex_);
    auto it = publishers_.find(topic);
    if (it == publishers_.end()) {
      auto qos = rclcpp::QoS(config_.queue_size);
      if (config_.sensor_data_best_effort && IsSensorDataMessage<MessageT>()) {
        qos.best_effort();
      }
      auto publisher = node_->create_publisher<MessageT>(topic, qos);
      it = publishers_.emplace(topic, publisher).first;
    }
    return std::dynamic_pointer_cast<rclcpp::Publisher<MessageT>>(it->second);
  }

  template <typename MessageT>
  void PublishMessage(const std::string& topic, MessageT message) {
    auto publisher = Publisher<MessageT>(topic);
    if (config_.publish_worker_threads > 0 && IsSensorDataMessage<MessageT>()) {
      auto shared_message = std::make_shared<MessageT>(std::move(message));
      EnqueuePublishTask([publisher, shared_message]() {
        publisher->publish(*shared_message);
      });
      return;
    }
    publisher->publish(message);
  }

  template <typename MessageT>
  static constexpr bool IsSensorDataMessage() {
    return std::is_same<MessageT, sensor_msgs::msg::CompressedImage>::value ||
           std::is_same<MessageT, sensor_msgs::msg::Image>::value ||
           std::is_same<MessageT, sensor_msgs::msg::Imu>::value ||
           std::is_same<MessageT, sensor_msgs::msg::PointCloud2>::value ||
           std::is_same<MessageT, livox_ros_driver2::msg::CustomMsg>::value;
  }

  void StartPublishWorkers() {
    StopPublishWorkers();
    if (config_.publish_worker_threads == 0) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(publish_queue_mutex_);
      publish_workers_stop_ = false;
      publish_queue_.clear();
      publish_tasks_dropped_ = 0;
    }
    publish_workers_.reserve(config_.publish_worker_threads);
    for (std::uint32_t i = 0; i < config_.publish_worker_threads; ++i) {
      publish_workers_.emplace_back([this]() { PublishWorkerLoop(); });
    }
  }

  void StopPublishWorkers() {
    {
      std::lock_guard<std::mutex> lock(publish_queue_mutex_);
      publish_workers_stop_ = true;
      publish_queue_.clear();
    }
    publish_queue_cv_.notify_all();
    for (auto& worker : publish_workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    publish_workers_.clear();
    {
      std::lock_guard<std::mutex> lock(publish_queue_mutex_);
      publish_workers_stop_ = false;
    }
  }

  void EnqueuePublishTask(std::function<void()> task) {
    {
      std::lock_guard<std::mutex> lock(publish_queue_mutex_);
      if (publish_workers_stop_) {
        return;
      }
      if (publish_queue_.size() >= config_.max_publish_queue_depth) {
        publish_queue_.pop_front();
        ++publish_tasks_dropped_;
        ROS_WARN_THROTTLE(
            2.0,
            "ROS publish queue full; dropped oldest sensor publish task "
            "(dropped=%llu depth=%zu)",
            static_cast<unsigned long long>(publish_tasks_dropped_),
            config_.max_publish_queue_depth);
      }
      publish_queue_.push_back(std::move(task));
    }
    publish_queue_cv_.notify_one();
  }

  void PublishWorkerLoop() {
    while (true) {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lock(publish_queue_mutex_);
        publish_queue_cv_.wait(lock, [this]() {
          return publish_workers_stop_ || !publish_queue_.empty();
        });
        if (publish_workers_stop_ && publish_queue_.empty()) {
          return;
        }
        task = std::move(publish_queue_.front());
        publish_queue_.pop_front();
      }
      task();
    }
  }

  void PublishDelivery(const orbvi_sdk::FrameDelivery& delivery) {
    const auto frame = delivery.frame.view();
    if (config_.log_delivery_stats &&
        (frame.stream_id == orbvi_sdk::StreamId::RawFisheye ||
         frame.stream_id == orbvi_sdk::StreamId::RectifiedFisheye)) {
      const auto& stats = delivery.stats;
      ROS_INFO_THROTTLE(
          2.0,
          "Host SDK delivery stream=%s observed=%.2fHz delivered=%llu dropped=%llu "
          "skipped_decodes=%llu decode_latency=%.2fms max_decode_latency=%.2fms "
          "receive_q=%zu decode_q=%zu overflow=%llu blocked=%s",
          orbvi_sdk::ToString(frame.stream_id),
          stats.observed_rate_hz,
          static_cast<unsigned long long>(stats.delivered_frames),
          static_cast<unsigned long long>(stats.dropped_frames),
          static_cast<unsigned long long>(stats.skipped_decodes),
          stats.decode_latency_ms,
          stats.max_decode_latency_ms,
          stats.receive_queue_depth,
          stats.decode_queue_depth,
          static_cast<unsigned long long>(stats.overflow_events),
          stats.blocked_reason.c_str());
    }
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
    if (config_.publish_compressed_images) {
      std::string skipped_reason;
      for (const auto& output :
           MakeCompressedImageMessages(frame, config_.split_multi_image_frames, &skipped_reason)) {
        PublishMessage(
            JoinTopic(config_.topic_prefix, output.topic_suffix),
            output.message);
      }
      if (!skipped_reason.empty()) {
        ROS_WARN_THROTTLE(5.0, "%s", skipped_reason.c_str());
      }
    }
    if (config_.publish_decoded_images) {
      const auto decoded_outputs =
          MakeDecodedImageMessages(frame.stream_id, delivery, config_.raw_camera_ids);
      if (decoded_outputs.empty()) {
        ROS_WARN_THROTTLE(
            5.0,
            "Decoded image output was requested, but Host SDK did not deliver decoded image data");
      }
      for (const auto& output : decoded_outputs) {
        PublishMessage(
            JoinTopic(config_.topic_prefix, output.topic_suffix),
            output.message);
      }
    }
  }

  void PublishImu(const orbvi_sdk::FrameView& frame) {
    sensor_msgs::msg::Imu message;
    if (!MakeImuMessage(frame, &message)) {
      ROS_WARN_THROTTLE(5.0, "Failed to parse Host SDK IMU payload");
      return;
    }
    const std::string suffix =
        frame.stream_id == orbvi_sdk::StreamId::LidarImu ? "lidar/imu" : "imu";
    PublishMessage(JoinTopic(config_.topic_prefix, suffix), std::move(message));
  }

  void PublishLivox(const orbvi_sdk::FrameView& frame) {
    livox_ros_driver2::msg::CustomMsg message;
    if (!MakeLivoxCustomMessage(frame, &message)) {
      ROS_WARN_THROTTLE(5.0, "Failed to parse Host SDK Livox payload");
      return;
    }
    PublishMessage(JoinTopic(config_.topic_prefix, "lidar/custom"), std::move(message));
  }

  void PublishDisparity(const orbvi_sdk::FrameView& frame) {
    sensor_msgs::msg::Image message;
    if (!MakeDisparityImage(frame, &message)) {
      ROS_WARN_THROTTLE(5.0, "Failed to parse Host SDK disparity payload");
      return;
    }
    PublishMessage(JoinTopic(config_.topic_prefix, "disparity"), std::move(message));
    PublishDepth(frame);
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
    sensor_msgs::msg::Image message;
    if (!MakeDepthImage(depth.view(), &message)) {
      ROS_WARN_THROTTLE(5.0, "Failed to convert ORBVI depth image to ROS Image");
      return;
    }
    PublishMessage(JoinTopic(config_.topic_prefix, "depth"), std::move(message));
    PublishDepthVisualization(depth.view(), frame);
  }

  void PublishDepthVisualization(
      const orbvi_sdk::DepthMapView& depth,
      const orbvi_sdk::FrameView& source_disparity) {
    if (config_.publish_depth_viz) {
      sensor_msgs::msg::Image message;
      if (MakeDepthVisualizationImage(
              depth,
              source_disparity,
              config_.depth_visualization_options,
              &message)) {
        PublishMessage(JoinTopic(config_.topic_prefix, "depth/viz"), std::move(message));
      } else {
        ROS_WARN_THROTTLE(5.0, "Failed to convert ORBVI depth image to visualization Image");
      }
    }
    if (config_.publish_depth_pointcloud && depth_calibration_) {
      auto cloud_result =
          orbvi_sdk::GeneratePointCloud(source_disparity, *depth_calibration_, config_.pointcloud_options);
      if (!cloud_result) {
        ROS_WARN_THROTTLE(
            5.0,
            "Failed to generate ORBVI SDK depth PointCloud: %s",
            cloud_result.error().message.c_str());
        return;
      }
      sensor_msgs::msg::PointCloud2 message;
      if (MakePointCloudMessage(cloud_result.value(), &message)) {
        PublishMessage(JoinTopic(config_.topic_prefix, "depth/points"), std::move(message));
      } else {
        ROS_WARN_THROTTLE(5.0, "Failed to convert ORBVI depth image to visualization PointCloud2");
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
    nav_msgs::msg::Odometry message;
    if (!MakeVioOdometry(frame, &message)) {
      ROS_WARN_THROTTLE(5.0, "Failed to parse Host SDK VIO payload");
      return;
    }
    Publisher<nav_msgs::msg::Odometry>(JoinTopic(config_.topic_prefix, suffix))
        ->publish(message);
  }

  rclcpp::Node::SharedPtr node_;
  ParameterReader private_node_;
  BridgeConfig config_;
  std::unique_ptr<orbvi_sdk::Client> client_;
  std::optional<orbvi_sdk::DepthCalibration> depth_calibration_;
  std::vector<orbvi_sdk::SubscriptionHandle> subscriptions_;
  std::mutex publish_queue_mutex_;
  std::condition_variable publish_queue_cv_;
  std::deque<std::function<void()>> publish_queue_;
  std::vector<std::thread> publish_workers_;
  bool publish_workers_stop_ = false;
  std::uint64_t publish_tasks_dropped_ = 0;
  std::mutex publishers_mutex_;
  std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;
};

BridgeNode::BridgeNode(rclcpp::Node::SharedPtr node)
    : impl_(new Impl(std::move(node))) {}

BridgeNode::~BridgeNode() = default;

bool BridgeNode::Start() {
  return impl_->Start();
}

void BridgeNode::Stop() {
  impl_->Stop();
}

}  // namespace orbvi_ros_bridge
