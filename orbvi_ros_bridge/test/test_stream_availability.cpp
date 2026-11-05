#include <gtest/gtest.h>
#include <limits>
#include "stream_availability.hpp"
#include "localization_streams.hpp"

namespace {
using orbvi_sdk::StreamId;
using orbvi_ros_bridge::StreamUnavailableReason;

TEST(LocalizationStreams, ExpandsLegacyGroupToSplitSubscriptionsWithoutDuplicates) {
  const std::vector<StreamId> requested = {
      StreamId::Imu, StreamId::VioPose, StreamId::OptimizedPose,
      StreamId::VioPose, StreamId::PredictedPose, StreamId::RegisteredCloud};
  const std::vector<StreamId> expected = {
      StreamId::Imu, StreamId::OptimizedPose, StreamId::PredictedPose,
      StreamId::RegisteredCloud};
  EXPECT_EQ(orbvi_ros_bridge::ResolveLocalizationStreams(requested), expected);
}

TEST(LocalizationStreams, ExplicitPoseStreamDoesNotEnableTheOtherStream) {
  for (const auto stream : {StreamId::PredictedPose, StreamId::OptimizedPose}) {
    const std::vector<StreamId> requested = {stream};
    EXPECT_EQ(orbvi_ros_bridge::ResolveLocalizationStreams(requested), requested);
  }
}

TEST(StreamAvailability, UnknownLidarUnitsDoNotDisableCameraOrMainImu) {
  EXPECT_FALSE(StreamUnavailableReason(StreamId::LidarImu, nullptr, nullptr).empty());
  for (const auto stream : {StreamId::RawFisheye, StreamId::RectifiedFisheye,
                            StreamId::Imu, StreamId::Disparity, StreamId::VioPose})
    EXPECT_TRUE(StreamUnavailableReason(stream, nullptr, nullptr).empty());
}

TEST(StreamAvailability, OptionalCloudUsesManifestAndFallsBackForLegacyFirmware) {
  orbvi_sdk::DeviceInfo device;
  device.supported_streams = {"raw_fisheye_stream", "imu_stream"};
  EXPECT_FALSE(StreamUnavailableReason(StreamId::RegisteredCloud, &device, nullptr).empty());
  device.supported_streams.push_back("registered_cloud_stream");
  EXPECT_TRUE(StreamUnavailableReason(StreamId::RegisteredCloud, &device, nullptr).empty());
  device.unsupported_streams = {"registered_cloud_stream"};
  EXPECT_FALSE(StreamUnavailableReason(StreamId::RegisteredCloud, &device, nullptr).empty());
  device = {};
  EXPECT_TRUE(StreamUnavailableReason(StreamId::RegisteredCloud, &device, nullptr).empty());
  EXPECT_TRUE(StreamUnavailableReason(StreamId::RegisteredCloud, nullptr, nullptr).empty());
}

TEST(StreamAvailability, StartupAndConversionShareSupportedUnitsAndFiniteScaleRules) {
  orbvi_sdk::LidarImuParameters parameters;
  parameters.linear_acceleration_scale_to_mps2 = 1;
  EXPECT_FALSE(orbvi_ros_bridge::ValidLidarImuParameters(&parameters));
  parameters.linear_acceleration_unit = static_cast<orbvi_sdk::LinearAccelerationUnit>(99);
  EXPECT_FALSE(orbvi_ros_bridge::ValidLidarImuParameters(&parameters));
  for (const auto unit : {orbvi_sdk::LinearAccelerationUnit::StandardGravity,
                          orbvi_sdk::LinearAccelerationUnit::MetersPerSecondSquared}) {
    parameters.linear_acceleration_unit = unit;
    for (const double scale : {0., -1., std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::max()}) {
      parameters.linear_acceleration_scale_to_mps2 = scale;
      EXPECT_FALSE(StreamUnavailableReason(StreamId::LidarImu, nullptr, &parameters).empty());
    }
    parameters.linear_acceleration_scale_to_mps2 = 9.80665;
    EXPECT_TRUE(StreamUnavailableReason(StreamId::LidarImu, nullptr, &parameters).empty());
  }
}
}  // namespace
