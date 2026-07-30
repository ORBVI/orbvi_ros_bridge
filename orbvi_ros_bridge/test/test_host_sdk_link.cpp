#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "frame_conversions.hpp"
#include "orbvi_sdk/client.hpp"
#include "panorama_options.hpp"

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
  EXPECT_TRUE(orbvi_ros_bridge::IsBridgePanoramaToken("pano"));
  EXPECT_TRUE(orbvi_ros_bridge::IsBridgePanoramaToken("pano_display_stream"));
  EXPECT_FALSE(orbvi_ros_bridge::ParseBridgeStream("pano", &stream));
}

TEST(OrbviRosBridgePanoramaImages, HostSdkPanoramaRoutesToImageTopic) {
  orbvi_sdk::OwnedPanoramaImage panorama;
  panorama.width = 2;
  panorama.height = 1;
  panorama.stride = 6;
  panorama.timestamp_ns = 123456789;
  panorama.frame_id = "orbvi/panorama/equirectangular";
  panorama.data = {1, 2, 3, 4, 5, 6};

  sensor_msgs::Image message;
  ASSERT_TRUE(orbvi_ros_bridge::MakePanoramaImageMessage(panorama.view(), &message));
  EXPECT_EQ(message.header.frame_id, "orbvi/panorama/equirectangular");
  EXPECT_EQ(message.width, 2u);
  EXPECT_EQ(message.height, 1u);
  EXPECT_EQ(message.encoding, "bgr8");
  EXPECT_EQ(message.step, 6u);
  EXPECT_EQ(message.data.size(), panorama.data.size());
}

TEST(OrbviRosBridgePanoramaImages, CopiesSdkNormalizedAzimuthWithoutMutation) {
  orbvi_sdk::OwnedPanoramaImage panorama;
  panorama.width = 4;
  panorama.height = 1;
  panorama.stride = 14;
  panorama.data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 90, 91};

  sensor_msgs::Image message;
  ASSERT_TRUE(orbvi_ros_bridge::MakePanoramaImageMessage(
      panorama.view(), &message));
  const std::vector<std::uint8_t> expected = {
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 90, 91};
  EXPECT_EQ(message.data, expected);
}

TEST(OrbviRosBridgePanoramaOptions, ParsesDynamicSeamModeTokens) {
  orbvi_sdk::PanoramaSeamMode mode = orbvi_sdk::PanoramaSeamMode::Fixed;

  EXPECT_TRUE(orbvi_ros_bridge::ParsePanoramaSeamMode("dynamic", &mode));
  EXPECT_EQ(mode, orbvi_sdk::PanoramaSeamMode::DynamicProgramming);

  mode = orbvi_sdk::PanoramaSeamMode::Fixed;
  EXPECT_TRUE(orbvi_ros_bridge::ParsePanoramaSeamMode("dynamic-programming", &mode));
  EXPECT_EQ(mode, orbvi_sdk::PanoramaSeamMode::DynamicProgramming);

  mode = orbvi_sdk::PanoramaSeamMode::Fixed;
  EXPECT_TRUE(orbvi_ros_bridge::ParsePanoramaSeamMode("dp", &mode));
  EXPECT_EQ(mode, orbvi_sdk::PanoramaSeamMode::DynamicProgramming);
}

TEST(OrbviRosBridgePanoramaOptions, RejectsUnknownSeamModeTokens) {
  orbvi_sdk::PanoramaSeamMode mode = orbvi_sdk::PanoramaSeamMode::Fixed;

  EXPECT_FALSE(orbvi_ros_bridge::ParsePanoramaSeamMode("unknown", &mode));
  EXPECT_EQ(mode, orbvi_sdk::PanoramaSeamMode::Fixed);
}

TEST(OrbviRosBridgePanoramaOptions, ParsesMode2BlendTokens) {
  orbvi_sdk::PanoramaBlendMode mode = orbvi_sdk::PanoramaBlendMode::Feather;

  EXPECT_TRUE(orbvi_ros_bridge::ParsePanoramaBlendMode("multiband", &mode));
  EXPECT_EQ(mode, orbvi_sdk::PanoramaBlendMode::MultiBand);

  mode = orbvi_sdk::PanoramaBlendMode::Feather;
  EXPECT_TRUE(orbvi_ros_bridge::ParsePanoramaBlendMode("mode2", &mode));
  EXPECT_EQ(mode, orbvi_sdk::PanoramaBlendMode::MultiBand);

  mode = orbvi_sdk::PanoramaBlendMode::Feather;
  EXPECT_TRUE(orbvi_ros_bridge::ParsePanoramaBlendMode("2", &mode));
  EXPECT_EQ(mode, orbvi_sdk::PanoramaBlendMode::MultiBand);
}

TEST(OrbviRosBridgePanoramaOptions, DepthAssistRequiresFixedSeamRoiMultiBand) {
  orbvi_sdk::PanoramaStitchOptions options;
  options.crop_top = 200;
  options.crop_bottom = 200;
  EXPECT_EQ(orbvi_ros_bridge::PanoramaOutputHeight(options), 624u);
  EXPECT_FALSE(orbvi_ros_bridge::SupportsDepthAssistedSeamRoi(options));

  options.blend_mode = orbvi_sdk::PanoramaBlendMode::MultiBand;
  EXPECT_TRUE(orbvi_ros_bridge::SupportsDepthAssistedSeamRoi(options));

  options.seam_ghost_suppression = true;
  EXPECT_FALSE(orbvi_ros_bridge::SupportsDepthAssistedSeamRoi(options));
  options.seam_ghost_suppression = false;
  options.seam_mode = orbvi_sdk::PanoramaSeamMode::DynamicProgramming;
  EXPECT_FALSE(orbvi_ros_bridge::SupportsDepthAssistedSeamRoi(options));
}

TEST(OrbviRosBridgeHostSdkLink, RigDepthPanoramaSymbolMatchesBundledHeaders) {
  orbvi_sdk::OwnedFrame disparity;
  orbvi_sdk::DepthCalibration calibration;
  const auto result = orbvi_sdk::GenerateRigDepthPanorama(disparity, calibration);
  EXPECT_FALSE(result);
}

TEST(OrbviRosBridgeDepthPanorama, UsesStereoPairTranslation) {
  orbvi_sdk::OwnedFrame disparity;
  disparity.header.stream_id = orbvi_sdk::StreamId::Disparity;
  disparity.header.format = orbvi_sdk::FrameFormat::Disparity32F;
  disparity.header.compression = orbvi_sdk::Compression::None;
  disparity.metadata["width"] = "2";
  disparity.metadata["height"] = "2";
  disparity.metadata["stride"] = "8";
  disparity.metadata["disparity_tile_count"] = "1";
  disparity.metadata["disparity_tile_width"] = "2";
  disparity.metadata["disparity_tile_height"] = "2";
  for (int i = 0; i < 4; ++i) {
    AppendFloat32(&disparity.payload, 1.0f);
  }
  disparity.header.payload_size =
      static_cast<std::uint32_t>(disparity.payload.size());

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
  tile.fx_px = 1.0;
  tile.fy_px = 1.0;
  tile.baseline_m = 1.0;
  tile.origin_translation_m = {1.0, 0.0, 0.0};
  tile.has_world_transform = true;
  calibration.tiles.push_back(tile);

  orbvi_sdk::RigDepthPanoramaOptions options;
  options.width = 64;
  options.height = 32;
  options.crop_top = 0;
  options.crop_bottom = 0;
  options.min_range_m = 0.1;
  options.max_range_m = 10.0;
  const auto generated =
      orbvi_sdk::GenerateRigDepthPanorama(disparity, calibration, options);
  ASSERT_TRUE(generated);
  EXPECT_GT(std::count_if(
                generated.value().confidence.begin(),
                generated.value().confidence.end(),
                [](std::uint8_t value) { return value != 0u; }),
            0);

  calibration.tiles[0].origin_translation_m = {0.0, 0.0, 0.0};
  const auto untranslated =
      orbvi_sdk::GenerateRigDepthPanorama(disparity, calibration, options);
  ASSERT_TRUE(untranslated);
  EXPECT_NE(generated.value().range_m, untranslated.value().range_m);
}

TEST(OrbviRosBridgeDepthPanorama, PublishedImageUsesCachedInverseLookup) {
  orbvi_sdk::OwnedFrame disparity;
  disparity.header.stream_id = orbvi_sdk::StreamId::Disparity;
  disparity.header.format = orbvi_sdk::FrameFormat::Disparity32F;
  disparity.header.compression = orbvi_sdk::Compression::None;
  disparity.metadata["width"] = "2";
  disparity.metadata["height"] = "2";
  disparity.metadata["stride"] = "8";
  disparity.metadata["disparity_tile_count"] = "1";
  disparity.metadata["disparity_tile_width"] = "2";
  disparity.metadata["disparity_tile_height"] = "2";
  for (int i = 0; i < 4; ++i) {
    AppendFloat32(&disparity.payload, 1.0f);
  }
  disparity.header.payload_size =
      static_cast<std::uint32_t>(disparity.payload.size());

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
  tile.fx_px = 1.0;
  tile.fy_px = 1.0;
  tile.cx_px = 0.5;
  tile.cy_px = 0.5;
  tile.baseline_m = 1.0;
  tile.origin_translation_m = {1.0, 0.0, 0.0};
  tile.has_world_transform = true;
  calibration.tiles.push_back(tile);

  orbvi_sdk::DepthPanoramaLookup lookup;
  std::string error;
  ASSERT_TRUE(
      orbvi_ros_bridge::BuildDepthPanoramaLookup(
          calibration, &lookup, &error))
      << error;
  sensor_msgs::Image translated;
  ASSERT_TRUE(
      orbvi_ros_bridge::MakeDepthPanoramaImage(
          disparity.view(), calibration, lookup, &translated, &error))
      << error;

  calibration.tiles[0].origin_translation_m = {0.0, 0.0, 0.0};
  sensor_msgs::Image untranslated;
  ASSERT_TRUE(
      orbvi_ros_bridge::MakeDepthPanoramaImage(
          disparity.view(), calibration, lookup, &untranslated, &error))
      << error;
  EXPECT_EQ(translated.data, untranslated.data);
  EXPECT_TRUE(std::any_of(
      translated.data.begin(),
      translated.data.end(),
      [](std::uint8_t value) { return value != 0u; }));
}

TEST(OrbviRosBridgeDepthPanorama, CopiesSdkAzimuthContractWithoutMutation) {
  orbvi_sdk::RigDepthPanorama panorama;
  panorama.width = 4;
  panorama.height = 1;
  panorama.full_height = 1;
  panorama.range_m = {1.0f, 2.0f, 3.0f, 4.0f};

  sensor_msgs::Image image;
  std::string error;
  ASSERT_TRUE(
      orbvi_ros_bridge::MakeDepthPanoramaImage(panorama, &image, &error))
      << error;
  ASSERT_EQ(image.data.size(), 4u * sizeof(float));
  std::array<float, 4> output = {};
  std::memcpy(output.data(), image.data.data(), image.data.size());
  EXPECT_EQ(output, (std::array<float, 4>{1.0f, 2.0f, 3.0f, 4.0f}));
  EXPECT_EQ(image.header.frame_id, "orbvi/depth/panorama");
}

TEST(OrbviRosBridgeDecodedImages, RectifiedCoreMetadataRoutesToPairDirectionTopic) {
  orbvi_sdk::OwnedDecodedImage decoded;
  decoded.width = 1;
  decoded.height = 1;
  decoded.stride = 3;
  decoded.timestamp_ns = 123456789;
  decoded.frame_id = "orbvi/rectified/left/right";
  decoded.camera_id = "left/right";
  decoded.data = {1, 2, 3};

  orbvi_sdk::FrameDelivery delivery;
  delivery.frame.metadata["pair_id"] = "2";
  delivery.frame.metadata["side"] = "right";
  delivery.frame.metadata["pair_direction"] = "rear";
  delivery.frame.metadata["source_camera_direction"] = "left";
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

TEST(OrbviRosBridgeDecodedImages, RectifiedBundleUsesPairDirectionForBothEyes) {
  const std::vector<std::string> directions = {"front", "right", "rear", "left"};
  orbvi_sdk::FrameDelivery delivery;
  delivery.frame.header.stream_id = orbvi_sdk::StreamId::RectifiedFisheye;
  delivery.frame.metadata["bundle_type"] = "RECTIFIED_FISHEYE_BUNDLE";
  for (std::size_t i = 0; i < 8u; ++i) {
    const std::size_t pair_id = i / 2u;
    const std::string side = (i % 2u == 0u) ? "left" : "right";
    const std::string legacy_direction =
        side == "left" ? directions[pair_id] : directions[(pair_id + 1u) % 4u];
    orbvi_sdk::OwnedDecodedImage decoded;
    decoded.width = 1;
    decoded.height = 1;
    decoded.stride = 3;
    decoded.frame_id = "orbvi/rectified/" + legacy_direction + "/" + side;
    decoded.camera_id = legacy_direction + "/" + side;
    decoded.data = {static_cast<std::uint8_t>(i), 0, 0};
    delivery.decoded_images.push_back(std::move(decoded));
    const std::string prefix = "rectified." + std::to_string(i) + ".";
    delivery.frame.metadata[prefix + "pair_id"] = std::to_string(pair_id);
    delivery.frame.metadata[prefix + "side"] = side;
  }

  const auto outputs = orbvi_ros_bridge::MakeDecodedImageMessages(
      orbvi_sdk::StreamId::RectifiedFisheye,
      delivery);
  ASSERT_EQ(outputs.size(), 8u);
  for (std::size_t i = 0; i < outputs.size(); ++i) {
    const std::size_t pair_id = i / 2u;
    const std::string side = (i % 2u == 0u) ? "left" : "right";
    EXPECT_EQ(
        outputs[i].topic_suffix,
        "rectified/" + directions[pair_id] + "/" + side + "/image");
    EXPECT_EQ(
        outputs[i].message.header.frame_id,
        "orbvi/rectified/" + directions[pair_id] + "/" + side);
    ASSERT_FALSE(outputs[i].message.data.empty());
    EXPECT_EQ(outputs[i].message.data.front(), static_cast<std::uint8_t>(i));
  }
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
    frame.metadata[prefix + "pair_direction"] =
        std::array<const char*, 4>{"front", "right", "rear", "left"}[i / 2];
    frame.metadata[prefix + "source_camera_direction"] =
        std::array<const char*, 4>{"front", "right", "rear", "left"}[
            (i / 2 + i % 2) % 4];
    frame.payload.push_back(static_cast<std::uint8_t>(i));
  }

  std::string skipped_reason;
  const auto outputs =
      orbvi_ros_bridge::MakeCompressedImageMessages(frame.view(), true, &skipped_reason);

  ASSERT_EQ(outputs.size(), 8u);
  EXPECT_TRUE(skipped_reason.empty());
  EXPECT_EQ(outputs[0].topic_suffix, "rectified/front/left/image/compressed");
  EXPECT_EQ(outputs[0].message.header.frame_id, "orbvi/rectified/front/left");
  EXPECT_EQ(outputs[1].topic_suffix, "rectified/front/right/image/compressed");
  EXPECT_EQ(outputs[1].message.header.frame_id, "orbvi/rectified/front/right");
  EXPECT_EQ(outputs[2].topic_suffix, "rectified/right/left/image/compressed");
  EXPECT_EQ(outputs[2].message.header.frame_id, "orbvi/rectified/right/left");
  EXPECT_EQ(outputs[3].topic_suffix, "rectified/right/right/image/compressed");
  EXPECT_EQ(outputs[3].message.header.frame_id, "orbvi/rectified/right/right");
  EXPECT_EQ(outputs[4].topic_suffix, "rectified/rear/left/image/compressed");
  EXPECT_EQ(outputs[4].message.header.frame_id, "orbvi/rectified/rear/left");
  EXPECT_EQ(outputs[5].topic_suffix, "rectified/rear/right/image/compressed");
  EXPECT_EQ(outputs[5].message.header.frame_id, "orbvi/rectified/rear/right");
  EXPECT_EQ(outputs[6].topic_suffix, "rectified/left/left/image/compressed");
  EXPECT_EQ(outputs[6].message.header.frame_id, "orbvi/rectified/left/left");
  EXPECT_EQ(outputs[7].topic_suffix, "rectified/left/right/image/compressed");
  EXPECT_EQ(outputs[7].message.header.frame_id, "orbvi/rectified/left/right");
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

TEST(OrbviRosBridgeCompressedImages, RejectsContradictoryCoreDirectionMetadata) {
  orbvi_sdk::OwnedFrame frame;
  frame.header.stream_id = orbvi_sdk::StreamId::RectifiedFisheye;
  frame.header.format = orbvi_sdk::FrameFormat::RectifiedMjpeg;
  frame.header.compression = orbvi_sdk::Compression::Jpeg;
  frame.header.timestamp_ns = 123456789;
  frame.frame_id = "orbvi/rectified/front/right";
  frame.metadata["pair_id"] = "0";
  frame.metadata["side"] = "right";
  frame.metadata["pair_direction"] = "rear";
  frame.metadata["source_camera_direction"] = "right";
  frame.payload = {0xFF, 0xD8, 0xFF, 0xD9};

  std::string skipped_reason;
  const auto outputs =
      orbvi_ros_bridge::MakeCompressedImageMessages(
          frame.view(), true, &skipped_reason);

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

TEST(OrbviRosBridgeVio, BuildsBoundedPathAndResetsOnDiscontinuity) {
  nav_msgs::Path path;
  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.orientation.w = 1.0;

  for (std::uint32_t index = 0; index < 5u; ++index) {
    odometry.header.stamp = ros::Time(index + 1u, 0u);
    odometry.pose.pose.position.x = static_cast<double>(index);
    ASSERT_TRUE(orbvi_ros_bridge::AppendVioOdometryToPath(
        odometry, 3u, &path));
  }

  ASSERT_EQ(path.poses.size(), 3u);
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 4.0);

  odometry.header.frame_id = "map_reset";
  odometry.header.stamp = ros::Time(1u, 0u);
  odometry.pose.pose.position.x = 10.0;
  ASSERT_TRUE(orbvi_ros_bridge::AppendVioOdometryToPath(
      odometry, 3u, &path));
  ASSERT_EQ(path.poses.size(), 1u);
  EXPECT_EQ(path.header.frame_id, "map_reset");
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, 10.0);

  odometry.header.stamp = ros::Time(0u, 0u);
  odometry.pose.pose.position.x = 11.0;
  ASSERT_TRUE(orbvi_ros_bridge::AppendVioOdometryToPath(
      odometry, 3u, &path));
  ASSERT_EQ(path.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, 11.0);
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

  sensor_msgs::Image image;
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

  sensor_msgs::Image image;
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

  sensor_msgs::Image image;
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

  sensor_msgs::Image image;
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

TEST(OrbviRosBridgeDepthImages, SplitVisualizationCanOverlayFiveByFiveMetricGrid) {
  orbvi_sdk::DepthMap depth;
  depth.pixel_format = orbvi_sdk::DepthPixelFormat::Float32Meters;
  depth.width = 100;
  depth.height = 100;
  depth.stride = depth.width * sizeof(float);
  depth.frame_id = "orbvi/disparity/depth";
  for (std::size_t index = 0; index < 10000; ++index) {
    AppendFloat32(&depth.data, 2.5f);
  }

  orbvi_sdk::OwnedFrame disparity;
  disparity.header.stream_id = orbvi_sdk::StreamId::Disparity;
  disparity.header.format = orbvi_sdk::FrameFormat::Disparity32F;
  disparity.header.compression = orbvi_sdk::Compression::None;
  disparity.frame_id = "orbvi/disparity";
  disparity.metadata["width"] = "100";
  disparity.metadata["height"] = "100";
  disparity.metadata["stride"] = "400";
  for (std::size_t index = 0; index < 10000; ++index) {
    AppendFloat32(&disparity.payload, 1.0f);
  }

  orbvi_sdk::DepthCalibration calibration;
  calibration.width = 100;
  calibration.height = 100;
  calibration.tile_count = 1;
  calibration.tile_width = 100;
  calibration.tile_height = 100;
  orbvi_sdk::DepthTileCalibration tile;
  tile.tile_index = 0;
  tile.width = 100;
  tile.height = 100;
  calibration.tiles.push_back(tile);

  orbvi_ros_bridge::DepthVisualizationOptions plain_options;
  plain_options.min_depth_m = 0.1;
  plain_options.max_depth_m = 20.0;
  const auto plain = orbvi_ros_bridge::MakeSplitDepthVisualizationImages(
      depth.view(), disparity.view(), plain_options, calibration);
  ASSERT_EQ(plain.size(), 1u);
  EXPECT_EQ(plain[0].topic_suffix, "front/depth/viz");

  auto grid_options = plain_options;
  grid_options.show_grid_values = true;
  const auto grid = orbvi_ros_bridge::MakeSplitDepthVisualizationImages(
      depth.view(), disparity.view(), grid_options, calibration);
  ASSERT_EQ(grid.size(), 1u);

  const std::size_t grid_line_offset =
      static_cast<std::size_t>(1) * grid[0].message.step + 16u * 3u;
  ASSERT_LT(grid_line_offset + 2u, grid[0].message.data.size());
  EXPECT_NE(plain[0].message.data[grid_line_offset + 0u], 255u);
  EXPECT_EQ(grid[0].message.data[grid_line_offset + 0u], 255u);
  EXPECT_EQ(grid[0].message.data[grid_line_offset + 1u], 255u);
  EXPECT_EQ(grid[0].message.data[grid_line_offset + 2u], 255u);
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

  sensor_msgs::PointCloud2 cloud;
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

  sensor_msgs::PointCloud2 message;
  ASSERT_TRUE(orbvi_ros_bridge::MakePointCloudMessage(cloud, &message));
  EXPECT_EQ(message.header.frame_id, "base_link");
  EXPECT_EQ(message.height, 1u);
  EXPECT_EQ(message.width, 1u);
  EXPECT_FALSE(message.is_dense);
  EXPECT_GT(message.point_step, 0u);
  EXPECT_EQ(message.data.size(), message.point_step * message.width);
}
