#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "frame_conversions_ros2.hpp"
#include "orbvi_sdk/client.hpp"

namespace {

void AppendFloat32(std::vector<std::uint8_t>* out, float value) {
  std::uint8_t bytes[sizeof(float)] = {};
  std::memcpy(bytes, &value, sizeof(value));
  out->insert(out->end(), bytes, bytes + sizeof(bytes));
}

bool Contains(const std::string& value, const std::string& needle) {
  return value.find(needle) != std::string::npos;
}

}  // namespace

TEST(OrbviRosBridgeHostSdkLink, ClientSymbolResolvesFromHostSdkLibrary) {
  orbvi_sdk::ClientOptions options;
  options.host = "127.0.0.1";
  orbvi_sdk::Client client(options);
  EXPECT_FALSE(client.connected());
}

TEST(OrbviRosBridgeHostSdkLink, PublicStreamParsingIsHostSdkApi) {
  orbvi_sdk::StreamId stream = orbvi_sdk::StreamId::Unknown;
  EXPECT_TRUE(orbvi_sdk::ParseStreamId("raw_fisheye_stream", &stream));
  EXPECT_EQ(stream, orbvi_sdk::StreamId::RawFisheye);
  EXPECT_STREQ(orbvi_sdk::ToString(stream), "raw_fisheye_stream");
}

TEST(OrbviRosBridgeDecodedImages, RectifiedPublicFrameIdRoutesToDirectionTopic) {
  orbvi_sdk::OwnedDecodedImage decoded;
  decoded.width = 1;
  decoded.height = 1;
  decoded.stride = 3;
  decoded.timestamp_ns = 123456789;
  decoded.frame_id = "orbvi/rectified/rear/right";
  decoded.camera_id = "rear/right";
  decoded.data = {1, 2, 3};

  orbvi_sdk::FrameDelivery delivery;
  delivery.decoded_image = decoded;
  delivery.decoded_images.push_back(decoded);

  const auto outputs = orbvi_ros_bridge::MakeDecodedImageMessages(
      orbvi_sdk::StreamId::RectifiedFisheye,
      delivery);
  ASSERT_EQ(outputs.size(), 1u);
  EXPECT_EQ(outputs.front().topic_suffix, "rectified/rear/right/image");
  EXPECT_EQ(outputs.front().message.header.frame_id, "orbvi/rectified/rear/right");
  EXPECT_EQ(outputs.front().message.width, 1u);
  EXPECT_EQ(outputs.front().message.height, 1u);
  EXPECT_EQ(outputs.front().message.encoding, "bgr8");
}

TEST(OrbviRosBridgeDecodedImages, InternalRectifiedPairFrameIdDoesNotPublish) {
  orbvi_sdk::OwnedDecodedImage decoded;
  decoded.width = 1;
  decoded.height = 1;
  decoded.stride = 3;
  decoded.timestamp_ns = 123456789;
  decoded.frame_id = "orbvi/rectified/pair_3/right";
  decoded.camera_id = "3";
  decoded.data = {1, 2, 3};

  orbvi_sdk::FrameDelivery delivery;
  delivery.decoded_images.push_back(decoded);

  const auto outputs = orbvi_ros_bridge::MakeDecodedImageMessages(
      orbvi_sdk::StreamId::RectifiedFisheye,
      delivery);
  EXPECT_TRUE(outputs.empty());
}

TEST(OrbviRosBridgeDecodedImages, InvalidRectifiedMetadataDoesNotPublishInternalViewTopic) {
  orbvi_sdk::OwnedDecodedImage decoded;
  decoded.width = 1;
  decoded.height = 1;
  decoded.stride = 3;
  decoded.timestamp_ns = 123456789;
  decoded.frame_id = "orbvi/rectified/view_7";
  decoded.camera_id = "7";
  decoded.data = {1, 2, 3};

  orbvi_sdk::FrameDelivery delivery;
  delivery.decoded_images.push_back(decoded);

  const auto outputs = orbvi_ros_bridge::MakeDecodedImageMessages(
      orbvi_sdk::StreamId::RectifiedFisheye,
      delivery);
  EXPECT_TRUE(outputs.empty());
}

TEST(OrbviRosBridgeCompressedImages, RectifiedBundleRoutesToDirectionTopics) {
  orbvi_sdk::OwnedFrame frame;
  frame.header.stream_id = orbvi_sdk::StreamId::RectifiedFisheye;
  frame.header.format = orbvi_sdk::FrameFormat::RectifiedMjpegBundle;
  frame.header.compression = orbvi_sdk::Compression::Mjpeg;
  frame.header.timestamp_ns = 123456789;
  frame.frame_id = "orbvi/rectified_fisheye";
  frame.metadata["bundle_type"] = "RECTIFIED_FISHEYE_BUNDLE";
  frame.metadata["image_count"] = "8";
  frame.metadata["entry_count"] = "8";
  for (int i = 0; i < 8; ++i) {
    const std::string prefix = "rectified." + std::to_string(i) + ".";
    frame.metadata[prefix + "payload_offset"] = std::to_string(i);
    frame.metadata[prefix + "payload_size"] = "1";
    frame.metadata[prefix + "timestamp_ns"] = "123456789";
    frame.metadata[prefix + "pair_id"] = std::to_string(i / 2);
    frame.metadata[prefix + "side"] = (i % 2 == 0) ? "left" : "right";
    frame.payload.push_back(static_cast<std::uint8_t>(i));
  }

  std::string skipped_reason;
  const auto outputs =
      orbvi_ros_bridge::MakeCompressedImageMessages(frame.view(), true, &skipped_reason);

  ASSERT_EQ(outputs.size(), 8u);
  EXPECT_TRUE(skipped_reason.empty());
  EXPECT_EQ(outputs[0].topic_suffix, "rectified/front/left/image/compressed");
  EXPECT_EQ(outputs[0].message.header.frame_id, "orbvi/rectified/front/left");
  EXPECT_EQ(outputs[1].topic_suffix, "rectified/right/right/image/compressed");
  EXPECT_EQ(outputs[1].message.header.frame_id, "orbvi/rectified/right/right");
  EXPECT_EQ(outputs[2].topic_suffix, "rectified/right/left/image/compressed");
  EXPECT_EQ(outputs[2].message.header.frame_id, "orbvi/rectified/right/left");
  EXPECT_EQ(outputs[3].topic_suffix, "rectified/rear/right/image/compressed");
  EXPECT_EQ(outputs[3].message.header.frame_id, "orbvi/rectified/rear/right");
  EXPECT_EQ(outputs[4].topic_suffix, "rectified/rear/left/image/compressed");
  EXPECT_EQ(outputs[4].message.header.frame_id, "orbvi/rectified/rear/left");
  EXPECT_EQ(outputs[5].topic_suffix, "rectified/left/right/image/compressed");
  EXPECT_EQ(outputs[5].message.header.frame_id, "orbvi/rectified/left/right");
  EXPECT_EQ(outputs[6].topic_suffix, "rectified/left/left/image/compressed");
  EXPECT_EQ(outputs[6].message.header.frame_id, "orbvi/rectified/left/left");
  EXPECT_EQ(outputs[7].topic_suffix, "rectified/front/right/image/compressed");
  EXPECT_EQ(outputs[7].message.header.frame_id, "orbvi/rectified/front/right");
  for (const auto& output : outputs) {
    EXPECT_FALSE(Contains(output.topic_suffix, "view_"));
    EXPECT_FALSE(Contains(output.message.header.frame_id, "pair_"));
  }
}

TEST(OrbviRosBridgeCompressedImages, SingleRectifiedFrameUsesDirectionFrameId) {
  orbvi_sdk::OwnedFrame frame;
  frame.header.stream_id = orbvi_sdk::StreamId::RectifiedFisheye;
  frame.header.format = orbvi_sdk::FrameFormat::RectifiedMjpeg;
  frame.header.compression = orbvi_sdk::Compression::Jpeg;
  frame.header.timestamp_ns = 123456789;
  frame.frame_id = "orbvi/rectified/rear/left";
  frame.metadata["pair_id"] = "2";
  frame.metadata["side"] = "left";
  frame.payload = {0xFF, 0xD8, 0xFF, 0xD9};

  std::string skipped_reason;
  const auto outputs =
      orbvi_ros_bridge::MakeCompressedImageMessages(frame.view(), true, &skipped_reason);

  ASSERT_EQ(outputs.size(), 1u);
  EXPECT_TRUE(skipped_reason.empty());
  EXPECT_EQ(outputs.front().topic_suffix, "rectified/rear/left/image/compressed");
  EXPECT_EQ(outputs.front().message.header.frame_id, "orbvi/rectified/rear/left");
}

TEST(OrbviRosBridgeCompressedImages, SingleRectifiedFrameRequiresPairSideMetadata) {
  orbvi_sdk::OwnedFrame frame;
  frame.header.stream_id = orbvi_sdk::StreamId::RectifiedFisheye;
  frame.header.format = orbvi_sdk::FrameFormat::RectifiedMjpeg;
  frame.header.compression = orbvi_sdk::Compression::Jpeg;
  frame.header.timestamp_ns = 123456789;
  frame.frame_id = "orbvi/rectified/pair_3/right";
  frame.payload = {0xFF, 0xD8, 0xFF, 0xD9};

  std::string skipped_reason;
  const auto outputs =
      orbvi_ros_bridge::MakeCompressedImageMessages(frame.view(), true, &skipped_reason);

  EXPECT_TRUE(outputs.empty());
}

TEST(OrbviRosBridgeCompressedImages, InvalidSingleRectifiedFrameDoesNotPublishInternalTopic) {
  orbvi_sdk::OwnedFrame frame;
  frame.header.stream_id = orbvi_sdk::StreamId::RectifiedFisheye;
  frame.header.format = orbvi_sdk::FrameFormat::RectifiedMjpeg;
  frame.header.compression = orbvi_sdk::Compression::Jpeg;
  frame.header.timestamp_ns = 123456789;
  frame.frame_id = "orbvi/rectified/unknown";
  frame.metadata["view_index"] = "99";
  frame.payload = {0xFF, 0xD8, 0xFF, 0xD9};

  std::string skipped_reason;
  const auto outputs =
      orbvi_ros_bridge::MakeCompressedImageMessages(frame.view(), true, &skipped_reason);

  EXPECT_TRUE(outputs.empty());
}

TEST(OrbviRosBridgeVio, RoutesOptimizedAndPropagatedSourcesToSeparateTopics) {
  orbvi_sdk::OwnedFrame frame;
  frame.header.stream_id = orbvi_sdk::StreamId::VioPose;
  frame.header.format = orbvi_sdk::FrameFormat::VioPose;
  frame.header.compression = orbvi_sdk::Compression::None;
  frame.frame_id = "orbvi/vio_pose_stream";

  frame.metadata["vio_source"] = "optimized";
  EXPECT_EQ(orbvi_ros_bridge::VioTopicSuffix(frame.view()), "vio/odometry");

  frame.metadata["vio_source"] = "propagated";
  EXPECT_EQ(orbvi_ros_bridge::VioTopicSuffix(frame.view()), "vio/imu_prediction");

  frame.metadata["vio_source"] = "unknown";
  EXPECT_TRUE(orbvi_ros_bridge::VioTopicSuffix(frame.view()).empty());
}

TEST(OrbviRosBridgeDepthImages, Float32MetersMapsToRosImage) {
  orbvi_sdk::DepthMap depth;
  depth.pixel_format = orbvi_sdk::DepthPixelFormat::Float32Meters;
  depth.width = 2;
  depth.height = 1;
  depth.stride = 8;
  depth.timestamp_ns = 123456789;
  depth.frame_id = "orbvi/disparity_stream/depth";
  depth.data = {0, 0, 128, 63, 0, 0, 0, 64};

  sensor_msgs::msg::Image image;
  ASSERT_TRUE(orbvi_ros_bridge::MakeDepthImage(depth.view(), &image));
  EXPECT_EQ(image.width, 2u);
  EXPECT_EQ(image.height, 1u);
  EXPECT_EQ(image.step, 8u);
  EXPECT_EQ(image.encoding, "32FC1");
  EXPECT_EQ(image.header.frame_id, "orbvi/disparity_stream/depth");
  EXPECT_EQ(image.data.size(), 8u);
}

TEST(OrbviRosBridgeDepthImages, Uint16MillimetersMapsToRosImage) {
  orbvi_sdk::DepthMap depth;
  depth.pixel_format = orbvi_sdk::DepthPixelFormat::Uint16Millimeters;
  depth.width = 2;
  depth.height = 1;
  depth.stride = 4;
  depth.frame_id = "orbvi/depth";
  depth.data = {232, 3, 208, 7};

  sensor_msgs::msg::Image image;
  ASSERT_TRUE(orbvi_ros_bridge::MakeDepthImage(depth.view(), &image));
  EXPECT_EQ(image.encoding, "16UC1");
  EXPECT_EQ(image.step, 4u);
  EXPECT_EQ(image.data, depth.data);
}

TEST(OrbviRosBridgeDepthImages, VisualizationImageUsesBgr8) {
  orbvi_sdk::DepthMap depth;
  depth.pixel_format = orbvi_sdk::DepthPixelFormat::Uint16Millimeters;
  depth.width = 3;
  depth.height = 1;
  depth.stride = 6;
  depth.frame_id = "orbvi/depth";
  depth.data = {232, 3, 16, 39, 0, 0};

  orbvi_ros_bridge::DepthVisualizationOptions options;
  options.min_depth_m = 0.0;
  options.max_depth_m = 10.0;

  sensor_msgs::msg::Image image;
  ASSERT_TRUE(orbvi_ros_bridge::MakeDepthVisualizationImage(depth.view(), options, &image));
  EXPECT_EQ(image.encoding, "bgr8");
  EXPECT_EQ(image.step, 9u);
  ASSERT_EQ(image.data.size(), 9u);
  EXPECT_GT(image.data[2], image.data[0]);
  EXPECT_GT(image.data[3], image.data[5]);
  EXPECT_EQ(image.data[6], 0u);
  EXPECT_EQ(image.data[7], 0u);
  EXPECT_EQ(image.data[8], 0u);
}

TEST(OrbviRosBridgeDepthImages, VisualizationImageMatchesSourceDisparityJet) {
  orbvi_sdk::DepthMap depth;
  depth.pixel_format = orbvi_sdk::DepthPixelFormat::Float32Meters;
  depth.width = 3;
  depth.height = 1;
  depth.stride = 12;
  depth.timestamp_ns = 123456789;
  depth.frame_id = "orbvi/disparity/depth";
  depth.data = {0, 0, 0, 64, 0, 0, 128, 63, 0, 0, 0, 63};

  orbvi_sdk::OwnedFrame disparity;
  disparity.header.stream_id = orbvi_sdk::StreamId::Disparity;
  disparity.header.format = orbvi_sdk::FrameFormat::Disparity32F;
  disparity.header.compression = orbvi_sdk::Compression::None;
  disparity.header.timestamp_ns = depth.timestamp_ns;
  disparity.frame_id = "orbvi/disparity";
  disparity.metadata["width"] = "3";
  disparity.metadata["height"] = "1";
  disparity.metadata["stride"] = "12";
  AppendFloat32(&disparity.payload, 1.0f);
  AppendFloat32(&disparity.payload, 2.0f);
  AppendFloat32(&disparity.payload, 3.0f);

  orbvi_ros_bridge::DepthVisualizationOptions options;
  options.min_depth_m = 0.0;
  options.max_depth_m = 10.0;

  sensor_msgs::msg::Image image;
  ASSERT_TRUE(orbvi_ros_bridge::MakeDepthVisualizationImage(
      depth.view(),
      disparity.view(),
      options,
      &image));
  EXPECT_EQ(image.encoding, "bgr8");
  EXPECT_EQ(image.header.frame_id, "orbvi/disparity");
  ASSERT_EQ(image.data.size(), 9u);
  EXPECT_GT(image.data[0], image.data[2]);
  EXPECT_GT(image.data[8], image.data[6]);
}

TEST(OrbviRosBridgeDepthImages, PointCloudUsesDepthCalibration) {
  orbvi_sdk::DepthMap depth;
  depth.pixel_format = orbvi_sdk::DepthPixelFormat::Uint16Millimeters;
  depth.width = 2;
  depth.height = 2;
  depth.stride = 4;
  depth.frame_id = "orbvi/depth";
  depth.data = {232, 3, 208, 7, 0, 0, 160, 15};

  orbvi_sdk::DepthCalibration calibration;
  calibration.width = 2;
  calibration.height = 2;
  calibration.tile_count = 1;
  calibration.tile_width = 2;
  calibration.tile_height = 2;
  orbvi_sdk::DepthTileCalibration tile;
  tile.tile_index = 0;
  tile.width = 2;
  tile.height = 2;
  tile.fx_px = 2.0;
  tile.fy_px = 2.0;
  tile.cx_px = 0.0;
  tile.cy_px = 0.0;
  tile.baseline_m = 0.1;
  calibration.tiles.push_back(tile);

  orbvi_ros_bridge::DepthVisualizationOptions options;
  options.min_depth_m = 0.0;
  options.max_depth_m = 10.0;
  options.pointcloud_stride = 1;

  sensor_msgs::msg::PointCloud2 cloud;
  ASSERT_TRUE(orbvi_ros_bridge::MakeDepthPointCloud(
      depth.view(),
      calibration,
      options,
      &cloud));
  EXPECT_EQ(cloud.header.frame_id, "orbvi/depth");
  EXPECT_EQ(cloud.height, 1u);
  EXPECT_EQ(cloud.width, 3u);
  EXPECT_FALSE(cloud.is_dense);
  EXPECT_GT(cloud.point_step, 0u);
  EXPECT_EQ(cloud.data.size(), cloud.point_step * cloud.width);
}

TEST(OrbviRosBridgeDepthImages, SdkPointCloudMapsToRosPointCloud2) {
  orbvi_sdk::PointCloud cloud;
  cloud.timestamp_ns = 123456789;
  cloud.frame_id = "base_link";
  orbvi_sdk::PointCloudPoint point;
  point.x = 1.0f;
  point.y = 2.0f;
  point.z = 3.0f;
  point.r = 255;
  point.g = 128;
  point.b = 64;
  cloud.points.push_back(point);

  sensor_msgs::msg::PointCloud2 message;
  ASSERT_TRUE(orbvi_ros_bridge::MakePointCloudMessage(cloud, &message));
  EXPECT_EQ(message.header.frame_id, "base_link");
  EXPECT_EQ(message.height, 1u);
  EXPECT_EQ(message.width, 1u);
  EXPECT_FALSE(message.is_dense);
  EXPECT_GT(message.point_step, 0u);
  EXPECT_EQ(message.data.size(), message.point_step * message.width);
}
