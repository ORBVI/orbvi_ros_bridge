#include <gtest/gtest.h>
#include "board_ipm.hpp"

namespace orbvi_ros_bridge {
TEST(BoardIpm, SdkMountingAndGpuProjectionMatchAuditedTf) {
  orbvi_sdk::RobotGroundProjection p;
  p.xyz={.2,.1,.8};p.quaternion=orbvi_camera_contract::ProjectionQuaternionFromRpy({3.141592653589793,0,.3});
  p.ground_plane={.1,.2,1,.3};
  auto setup=ConfigureBoardTf(p,true,"");
  ASSERT_TRUE(setup.mounting);EXPECT_EQ("orbvi_link",setup.device_frame);
  EXPECT_EQ(p.xyz,setup.mounting->base_from_device.translation);
  EXPECT_EQ(p.quaternion,setup.mounting->base_from_device.quaternion);
  ResolvedRobotTf tf;tf.base_from_device=setup.mounting->base_from_device;tf.cameras.camera_hash="hash";
  orbvi_sdk::MetadataMap m{{"projection","robot_ground_plane_ipm"},{"gpu_projected","true"},
      {"cpu_fallback_used","false"},{"robot_projection_v1",orbvi_camera_contract::SerializeRobotGroundProjection(p)}};
  orbvi_sdk::FrameView frame;frame.stream_id=orbvi_sdk::StreamId::BevDisplay;
  frame.frame_id="base_link";frame.calibration_hash="hash";frame.metadata=&m;
  EXPECT_TRUE(CheckBoardRobotIpm(frame,tf,p.ground_plane).empty());
  for(double& q:tf.base_from_device.quaternion)q=-q;
  EXPECT_TRUE(CheckBoardRobotIpm(frame,tf,{.2,.4,2,.6}).empty());
  tf.base_from_device.translation[0]+=.01;
  EXPECT_FALSE(CheckBoardRobotIpm(frame,tf,p.ground_plane).empty());
  tf.base_from_device.translation=p.xyz;frame.calibration_hash="stale";
  EXPECT_FALSE(CheckBoardRobotIpm(frame,tf,p.ground_plane).empty());
  frame.calibration_hash="hash";
  EXPECT_FALSE(CheckBoardRobotIpm(frame,tf,{0,0,1,0}).empty());
  m["projection"]="ground_plane_ipm";
  EXPECT_FALSE(CheckBoardRobotIpm(frame,tf,p.ground_plane).empty());
}
TEST(BoardIpm, RejectsSdkFrameCollisions) {
  orbvi_sdk::RobotGroundProjection p;
  EXPECT_THROW(ConfigureBoardTf(p,true,"wrong_depth"),std::runtime_error);
  p.child_frame="mid360_imu";
  EXPECT_THROW(ConfigureBoardTf(p,true,""),std::runtime_error);
}
} // namespace orbvi_ros_bridge

TEST(BoardIpmDelivery, RejectsStaleOrRelabeledDecodedImage) {
  orbvi_sdk::FrameDelivery delivery;
  delivery.frame.header.timestamp_ns=1234;delivery.frame.frame_id="base_link";
  EXPECT_FALSE(orbvi_ros_bridge::BoardIpmDecodedFrameMatches(delivery));
  delivery.decoded_image.emplace();
  delivery.decoded_image->timestamp_ns=1234;delivery.decoded_image->frame_id="base_link";
  EXPECT_TRUE(orbvi_ros_bridge::BoardIpmDecodedFrameMatches(delivery));
  delivery.decoded_image->timestamp_ns=1233;
  EXPECT_FALSE(orbvi_ros_bridge::BoardIpmDecodedFrameMatches(delivery));
  delivery.decoded_image->timestamp_ns=1234;delivery.decoded_image->frame_id="orbvi_link";
  EXPECT_FALSE(orbvi_ros_bridge::BoardIpmDecodedFrameMatches(delivery));
}
