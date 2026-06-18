#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "orbvi_sdk/client.hpp"

namespace orbvi_ros_bridge {

template <typename MessageT>
struct TopicMessage {
  std::string topic_suffix;
  MessageT message;
};

using CompressedImageOutput = TopicMessage<sensor_msgs::CompressedImage>;
using ImageOutput = TopicMessage<sensor_msgs::Image>;

struct DepthVisualizationOptions {
  double min_depth_m = 0.1;
  double max_depth_m = 20.0;
  std::uint32_t pointcloud_stride = 4;
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
    const orbvi_sdk::FrameDelivery& delivery);
bool MakePanoramaImageMessage(
    const orbvi_sdk::PanoramaImageView& panorama,
    sensor_msgs::Image* out);

bool MakeDisparityImage(const orbvi_sdk::FrameView& frame, sensor_msgs::Image* out);
bool MakeDepthImage(const orbvi_sdk::DepthMapView& depth, sensor_msgs::Image* out);
bool MakeDepthVisualizationImage(
    const orbvi_sdk::DepthMapView& depth,
    const DepthVisualizationOptions& options,
    sensor_msgs::Image* out);
bool MakeDepthVisualizationImage(
    const orbvi_sdk::DepthMapView& depth,
    const orbvi_sdk::FrameView& source_disparity,
    const DepthVisualizationOptions& options,
    sensor_msgs::Image* out);
bool MakeDepthPointCloud(
    const orbvi_sdk::DepthMapView& depth,
    const orbvi_sdk::DepthCalibration& calibration,
    const DepthVisualizationOptions& options,
    sensor_msgs::PointCloud2* out);
bool MakePointCloudMessage(
    const orbvi_sdk::PointCloud& cloud,
    sensor_msgs::PointCloud2* out);
bool MakeImuMessage(const orbvi_sdk::FrameView& frame, sensor_msgs::Imu* out);
std::string VioTopicSuffix(const orbvi_sdk::FrameView& frame);
bool MakeLivoxCustomMessage(
    const orbvi_sdk::FrameView& frame,
    livox_ros_driver2::CustomMsg* out);
bool MakeVioOdometry(const orbvi_sdk::FrameView& frame, nav_msgs::Odometry* out);

}  // namespace orbvi_ros_bridge
