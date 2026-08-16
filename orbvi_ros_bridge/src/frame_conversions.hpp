#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
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
  bool show_grid_values = false;
  std::uint32_t pointcloud_stride = 4;
};

std::vector<std::string> SplitCsv(const std::string& value);
std::string NormalizeTopicPrefix(std::string prefix);
std::string JoinTopic(const std::string& prefix, std::string suffix);
bool ParseBridgeStream(const std::string& input, orbvi_sdk::StreamId* out);
bool IsBridgeImageStream(orbvi_sdk::StreamId stream);
bool IsBridgePanoramaToken(const std::string& input);
bool IsBridgeBevToken(const std::string& input);
bool ShouldFallbackToHostDisplay(const orbvi_sdk::SdkError& error);

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
bool MakeBoardDisplayImageMessage(
    const orbvi_sdk::FrameDelivery& delivery,
    const std::string& fallback_frame_id,
    sensor_msgs::Image* out);

bool MakeDisparityImage(const orbvi_sdk::FrameView& frame, sensor_msgs::Image* out);
std::vector<ImageOutput> MakeSplitDisparityImages(
    const orbvi_sdk::FrameView& frame,
    const orbvi_sdk::DepthCalibration& calibration);
bool MakeDepthImage(const orbvi_sdk::DepthMapView& depth, sensor_msgs::Image* out);
std::vector<ImageOutput> MakeSplitDepthImages(
    const orbvi_sdk::DepthMapView& depth,
    const orbvi_sdk::DepthCalibration& calibration);
bool BuildDepthPanoramaLookup(
    const orbvi_sdk::DepthCalibration& calibration,
    orbvi_sdk::DepthPanoramaLookup* out,
    std::string* error);
bool MakeDepthPanoramaImage(
    const orbvi_sdk::FrameView& disparity_frame,
    const orbvi_sdk::DepthCalibration& calibration,
    const orbvi_sdk::DepthPanoramaLookup& lookup,
    sensor_msgs::Image* out,
    std::string* error);
bool MakeDepthPanoramaImage(
    const orbvi_sdk::RigDepthPanorama& depth,
    sensor_msgs::Image* out,
    std::string* error);
bool MakeDepthPanoramaVisualizationImage(
    const sensor_msgs::Image& depth_panorama,
    sensor_msgs::Image* out);
bool MakeDepthVisualizationImage(
    const orbvi_sdk::DepthMapView& depth,
    const DepthVisualizationOptions& options,
    sensor_msgs::Image* out);
bool MakeDepthVisualizationImage(
    const orbvi_sdk::DepthMapView& depth,
    const orbvi_sdk::FrameView& source_disparity,
    const DepthVisualizationOptions& options,
    sensor_msgs::Image* out);
std::vector<ImageOutput> MakeSplitDepthVisualizationImages(
    const orbvi_sdk::DepthMapView& depth,
    const orbvi_sdk::FrameView& source_disparity,
    const DepthVisualizationOptions& options,
    const orbvi_sdk::DepthCalibration& calibration);
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
bool MakeLivoxPointCloud2(
    const livox_ros_driver2::CustomMsg& msg,
    sensor_msgs::PointCloud2* out);
bool MakeVioOdometry(const orbvi_sdk::FrameView& frame, nav_msgs::Odometry* out);
bool AppendVioOdometryToPath(
    const nav_msgs::Odometry& odometry,
    std::size_t max_poses,
    nav_msgs::Path* path);

}  // namespace orbvi_ros_bridge
