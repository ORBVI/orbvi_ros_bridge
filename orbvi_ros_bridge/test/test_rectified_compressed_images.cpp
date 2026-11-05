#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "orbvi_sdk/uvc_contract.hpp"

#if ORBVI_ROS_BRIDGE_TEST_ROS_VERSION == 2
#include "frame_conversions_ros2.hpp"
#else
#include "frame_conversions.hpp"
#endif

namespace {
constexpr std::uint64_t kStamp = 1700000000000000123ULL;
const std::array<std::string, 4> kDirections{{"front", "right", "rear", "left"}};

orbvi_sdk::OwnedFrame MakeAtlas() {
  orbvi_sdk::OwnedFrame frame;
  frame.header.stream_id = orbvi_sdk::StreamId::RectifiedFisheye;
  frame.header.format = orbvi_sdk::FrameFormat::RectifiedMjpegBundle;
  frame.header.compression = orbvi_sdk::Compression::Mjpeg;
  frame.header.timestamp_ns = kStamp;
  frame.frame_id = "orbvi/rectified_fisheye";
  frame.metadata = {{"bundle_type", "RECTIFIED_FISHEYE_ATLAS_BUNDLE_V2"},
                    {"layout", "side_major_2x4_320x256"}, {"image_count", "8"},
                    {"entry_count", "2"}, {"atlas_count", "2"},
                    {"width", "320"}, {"height", "256"},
                    {"atlas_width", "320"}, {"atlas_height", "1024"}};
  for (int side = 0; side < 2; ++side) {
    // Distinct monochrome textures for all eight logical views. Monochrome
    // avoids chroma upsampling across crop boundaries in the pixel comparison.
    cv::Mat atlas(1024, 320, CV_8UC3);
    for (int y = 0; y < atlas.rows; ++y) {
      for (int x = 0; x < atlas.cols; ++x) {
        const auto value = static_cast<std::uint8_t>(20 + side*100 + (y/256)*25 + (x+y)%20);
        atlas.at<cv::Vec3b>(y, x) = {value, value, value};
      }
    }
    std::vector<std::uint8_t> jpeg;
    if (!cv::imencode(".jpg", atlas, jpeg)) return {};
    const auto prefix = "rectified_atlas." + std::to_string(side) + ".";
    frame.metadata[prefix + "side"] = side ? "right" : "left";
    frame.metadata[prefix + "width"] = "320";
    frame.metadata[prefix + "height"] = "1024";
    frame.metadata[prefix + "timestamp_ns"] = std::to_string(kStamp);
    frame.metadata[prefix + "payload_offset"] = std::to_string(frame.payload.size());
    frame.metadata[prefix + "payload_size"] = std::to_string(jpeg.size());
    frame.payload.insert(frame.payload.end(), jpeg.begin(), jpeg.end());
  }
  for (int i = 0; i < 8; ++i) {
    const int pair = i/2, side = i%2;
    const auto prefix = "rectified." + std::to_string(i) + ".";
    frame.metadata[prefix + "pair_id"] = std::to_string(pair);
    frame.metadata[prefix + "side"] = side ? "right" : "left";
    frame.metadata[prefix + "pair_direction"] = kDirections[pair];
    frame.metadata[prefix + "source_camera_direction"] = kDirections[(pair+side)%4];
    frame.metadata[prefix + "atlas_index"] = std::to_string(side);
    frame.metadata[prefix + "source_roi_x"] = "0";
    frame.metadata[prefix + "source_roi_y"] = std::to_string(pair*256);
    frame.metadata[prefix + "width"] = "320";
    frame.metadata[prefix + "height"] = "256";
    // Deliberately differ from the aggregate frame timestamp.
    frame.metadata[prefix + "timestamp_ns"] = std::to_string(kStamp + i + 1);
  }
  frame.metadata["payload_size"] = std::to_string(frame.payload.size());
  return frame;
}

TEST(RectifiedCompressedAtlas, PublishesEightCorrectOwnedJpegsWithSourceStamps) {
  auto frame = MakeAtlas();
  const auto original = frame.payload;
  std::array<cv::Mat, 2> decoded_atlases;
  for (int side = 0; side < 2; ++side) {
    const auto prefix = "rectified_atlas." + std::to_string(side) + ".";
    const auto offset = std::stoul(frame.metadata.at(prefix + "payload_offset"));
    const auto size = std::stoul(frame.metadata.at(prefix + "payload_size"));
    std::vector<std::uint8_t> jpeg(frame.payload.begin()+offset, frame.payload.begin()+offset+size);
    decoded_atlases[side] = cv::imdecode(jpeg, cv::IMREAD_COLOR);
    ASSERT_FALSE(decoded_atlases[side].empty());
  }
  std::string reason;
  const auto outputs = orbvi_ros_bridge::MakeCompressedImageMessages(frame.view(), true, &reason);
  ASSERT_EQ(outputs.size(), 8u) << reason;
  EXPECT_TRUE(reason.empty());
  EXPECT_EQ(frame.payload, original);
  frame.payload.clear(); // Published JPEGs must outlive the input frame.
  frame.payload.shrink_to_fit();
  for (int i = 0; i < 8; ++i) {
    const std::string identity = kDirections[i/2] + (i%2 ? "/right" : "/left");
    const auto& message = outputs[i].message;
    EXPECT_EQ(outputs[i].topic_suffix, "rectified/" + identity + "/image/compressed");
    EXPECT_EQ(message.header.frame_id, "orbvi/rectified/" + identity);
#if ORBVI_ROS_BRIDGE_TEST_ROS_VERSION == 2
    const auto stamp = static_cast<std::uint64_t>(message.header.stamp.sec)*1000000000ULL +
                       message.header.stamp.nanosec;
#else
    const auto stamp = message.header.stamp.toNSec();
#endif
    EXPECT_EQ(stamp, kStamp + i + 1);
    EXPECT_EQ(message.format, "jpeg");
    const auto decoded = cv::imdecode(message.data, cv::IMREAD_COLOR);
    ASSERT_EQ(decoded.cols, 320);
    ASSERT_EQ(decoded.rows, 256);
    const auto expected = decoded_atlases[i%2](cv::Rect(0, (i/2)*256, 320, 256));
    EXPECT_EQ(cv::norm(decoded, expected, cv::NORM_INF), 0) << identity;
  }
}

TEST(RectifiedCompressedAtlas, DoesNotPublishAnUnsplitAtlasAsALogicalImage) {
  const auto frame = MakeAtlas();
  std::string reason;
  EXPECT_TRUE(orbvi_ros_bridge::MakeCompressedImageMessages(frame.view(), false, &reason).empty());
  EXPECT_FALSE(reason.empty());
}

TEST(RectifiedCompressedAtlas, SingleAtlasUvcAndBulkPreserveAllEightViews) {
  for (const auto* profile : {&orbvi_sdk::kUvcRectifiedAtlasProfileV1,
                              &orbvi_sdk::kUvcRectifiedAtlasProfileV2}) {
    const int width = profile->width, height = profile->height;
    const int tile_width = width / 2, tile_height = height / 4;
    cv::Mat atlas(height, width, CV_8UC3);
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x) {
        const auto value = static_cast<std::uint8_t>(20 +
            (y / tile_height * 2 + x / tile_width) * 25 + x % 16);
        atlas.at<cv::Vec3b>(y, x) = {value, value, value};
      }
    orbvi_sdk::OwnedFrame frame;
    frame.header.stream_id = orbvi_sdk::StreamId::RectifiedFisheye;
    frame.header.format = orbvi_sdk::FrameFormat::RectifiedMjpegBundle;
    frame.header.compression = orbvi_sdk::Compression::Mjpeg;
    frame.header.timestamp_ns = kStamp;
    ASSERT_TRUE(cv::imencode(".jpg", atlas, frame.payload));
    const auto decoded_atlas = cv::imdecode(frame.payload, cv::IMREAD_COLOR);
    frame.metadata = {
        {"layout", "rectified_atlas_2x4_pair_major_left_right"},
        {"transport", profile == &orbvi_sdk::kUvcRectifiedAtlasProfileV1 ? "uvc" : "vendor-bulk"},
        {"format_profile_id", std::to_string(profile->format_profile_id)},
        {"width", std::to_string(width)}, {"height", std::to_string(height)},
        {"payload_size", std::to_string(frame.payload.size())}};
    std::string reason;
    const auto outputs = orbvi_ros_bridge::MakeCompressedImageMessages(frame.view(), true, &reason);
    ASSERT_EQ(outputs.size(), 8u) << reason;
    frame.payload.clear();
    frame.payload.shrink_to_fit();
    for (int i = 0; i < 8; ++i) {
      const auto identity = kDirections[i / 2] + (i % 2 ? "/right" : "/left");
      EXPECT_EQ(outputs[i].topic_suffix, "rectified/" + identity + "/image/compressed");
      EXPECT_EQ(outputs[i].message.header.frame_id, "orbvi/rectified/" + identity);
      const auto actual = cv::imdecode(outputs[i].message.data, cv::IMREAD_COLOR);
      ASSERT_EQ(actual.cols, tile_width);
      ASSERT_EQ(actual.rows, tile_height);
      EXPECT_EQ(cv::norm(actual, decoded_atlas(cv::Rect(
          i % 2 * tile_width, i / 2 * tile_height, tile_width, tile_height)), cv::NORM_INF), 0);
    }
    EXPECT_TRUE(orbvi_ros_bridge::MakeCompressedImageMessages(frame.view(), false, &reason).empty());
    EXPECT_FALSE(reason.empty());
  }
}

TEST(RectifiedCompressedAtlas, UnsupportedFramesExplainStreamAndBundle) {
  auto frame = MakeAtlas();
  frame.metadata["bundle_type"] = "RECTIFIED_FISHEYE_ATLAS_BUNDLE_V3";
  std::string reason;
  EXPECT_TRUE(orbvi_ros_bridge::MakeCompressedImageMessages(frame.view(), true, &reason).empty());
  EXPECT_NE(reason.find("rectified_fisheye_stream"), std::string::npos);
  EXPECT_NE(reason.find("RECTIFIED_FISHEYE_ATLAS_BUNDLE_V3"), std::string::npos);
}

TEST(RectifiedCompressedAtlas, RejectsInvalidMetadataBoundsAndCorruptJpeg) {
  const auto valid = MakeAtlas();
  for (int corruption = 0; corruption < 5; ++corruption) {
    auto frame = valid;
    if (corruption == 0) frame.metadata["rectified.1.pair_direction"] = "rear";
    if (corruption == 1) frame.metadata["rectified_atlas.1.payload_size"] = "9999999";
    if (corruption == 2) frame.metadata["rectified.7.source_roi_y"] = "1024";
    if (corruption == 3) frame.metadata.erase("rectified.0.timestamp_ns");
    // Corrupt the second atlas: no partial first-atlas output may escape.
    if (corruption == 4) {
      const auto offset = std::stoul(frame.metadata.at("rectified_atlas.1.payload_offset"));
      frame.payload[offset] = 0;
    }
    std::string reason;
    EXPECT_TRUE(orbvi_ros_bridge::MakeCompressedImageMessages(frame.view(), true, &reason).empty())
        << corruption;
    EXPECT_FALSE(reason.empty()) << corruption;
  }
}
}  // namespace
