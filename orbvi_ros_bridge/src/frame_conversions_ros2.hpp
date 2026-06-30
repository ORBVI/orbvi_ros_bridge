#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "orbvi_sdk/client.hpp"

namespace orbvi_ros_bridge {

template <typename MessageT>
struct TopicMessage {
  std::string topic_suffix;
  MessageT message;
};

using CompressedImageOutput = TopicMessage<sensor_msgs::msg::CompressedImage>;
using ImageOutput = TopicMessage<sensor_msgs::msg::Image>;

struct DepthVisualizationOptions {
  double min_depth_m = 0.1;
  double max_depth_m = 20.0;
  std::uint32_t pointcloud_stride = 4;
};

struct DepthPanoramaSample {
  std::int32_t tile_index = -1;
  std::uint32_t x = 0;
  std::uint32_t y = 0;
  float range_scale = 0.0f;
};

struct DepthPanoramaLookup {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t full_height = 0;
  std::uint32_t crop_top = 0;
  std::uint32_t crop_bottom = 0;
  std::string calibration_version;
  std::string calibration_hash;
  std::vector<DepthPanoramaSample> samples;
};

std::vector<std::string> SplitCsv(const std::string& value);
std::string NormalizeTopicPrefix(std::string prefix);
std::string JoinTopic(const std::string& prefix, std::string suffix);
bool ParseBridgeStream(const std::string& input, orbvi_sdk::StreamId* out);
bool IsBridgeImageStream(orbvi_sdk::StreamId stream);
bool IsBridgePanoramaToken(const std::string& input);

std::vector<CompressedImageOutput> MakeCompressedImageMessages(
    const orbvi_sdk::FrameView& frame,
    bool split_bundles,
    std::string* skipped_reason);

std::vector<ImageOutput> MakeDecodedImageMessages(
    orbvi_sdk::StreamId stream,
    const orbvi_sdk::FrameDelivery& delivery,
    const std::vector<std::string>& raw_camera_ids = {});
bool MakePanoramaImageMessage(
    const orbvi_sdk::PanoramaImageView& panorama,
    sensor_msgs::msg::Image* out);

bool MakeDisparityImage(const orbvi_sdk::FrameView& frame, sensor_msgs::msg::Image* out);
std::vector<ImageOutput> MakeSplitDisparityImages(
    const orbvi_sdk::FrameView& frame,
    const orbvi_sdk::DepthCalibration& calibration);
bool MakeDepthImage(const orbvi_sdk::DepthMapView& depth, sensor_msgs::msg::Image* out);
std::vector<ImageOutput> MakeSplitDepthImages(
    const orbvi_sdk::DepthMapView& depth,
    const orbvi_sdk::DepthCalibration& calibration);
bool BuildDepthPanoramaLookup(
    const orbvi_sdk::DepthCalibration& calibration,
    DepthPanoramaLookup* out,
    std::string* error);
bool MakeDepthPanoramaImage(
    const orbvi_sdk::FrameView& disparity_frame,
    const orbvi_sdk::DepthCalibration& calibration,
    const DepthPanoramaLookup& lookup,
    sensor_msgs::msg::Image* out,
    std::string* error);
bool MakeDepthPanoramaVisualizationImage(
    const sensor_msgs::msg::Image& depth_panorama,
    sensor_msgs::msg::Image* out);
bool MakeDepthVisualizationImage(
    const orbvi_sdk::DepthMapView& depth,
    const DepthVisualizationOptions& options,
    sensor_msgs::msg::Image* out);
bool MakeDepthVisualizationImage(
    const orbvi_sdk::DepthMapView& depth,
    const orbvi_sdk::FrameView& source_disparity,
    const DepthVisualizationOptions& options,
    sensor_msgs::msg::Image* out);
std::vector<ImageOutput> MakeSplitDepthVisualizationImages(
    const orbvi_sdk::DepthMapView& depth,
    const orbvi_sdk::FrameView& source_disparity,
    const DepthVisualizationOptions& options,
    const orbvi_sdk::DepthCalibration& calibration);
bool MakeDepthPointCloud(
    const orbvi_sdk::DepthMapView& depth,
    const orbvi_sdk::DepthCalibration& calibration,
    const DepthVisualizationOptions& options,
    sensor_msgs::msg::PointCloud2* out);
bool MakePointCloudMessage(
    const orbvi_sdk::PointCloud& cloud,
    sensor_msgs::msg::PointCloud2* out);
bool MakeImuMessage(const orbvi_sdk::FrameView& frame, sensor_msgs::msg::Imu* out);
std::string VioTopicSuffix(const orbvi_sdk::FrameView& frame);
bool MakeLivoxCustomMessage(
    const orbvi_sdk::FrameView& frame,
    livox_ros_driver2::msg::CustomMsg* out);
bool MakeVioOdometry(const orbvi_sdk::FrameView& frame, nav_msgs::msg::Odometry* out);

}  // namespace orbvi_ros_bridge
