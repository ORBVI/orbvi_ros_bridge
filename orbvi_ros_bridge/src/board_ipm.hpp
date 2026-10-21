#pragma once

#include "robot_tf_session.hpp"
#include "orbvi_sdk/board_bev.hpp"
#include "orbvi_sdk/client.hpp"
#include <algorithm>
#include <condition_variable>

namespace orbvi_ros_bridge {

// One bounded startup subscription, stopped before live publishers are created.
// The board profile is the sole mounting/plane source with mounting_config=sdk.
inline orbvi_sdk::RobotGroundProjection ReadBoardMountingSnapshot(
    orbvi_sdk::Client& client, orbvi_sdk::SubscribeOptions options) {
  struct State {
    std::mutex mutex;std::condition_variable ready;
    bool received=false;
    std::optional<orbvi_sdk::RobotGroundProjection> projection;
    std::string error;
  };
  auto state=std::make_shared<State>();
  options.decode_policy.mode=orbvi_sdk::DecodeMode::RawOnly;
  auto subscription=client.subscribe(orbvi_sdk::StreamId::BevDisplay,options,
      [state](const orbvi_sdk::FrameDelivery& delivery) {
        auto result=orbvi_sdk::ReadBoardRobotProjection(delivery.frame.view());
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->received) return;
        if(result) state->projection=result.value(); else state->error=result.error().message;
        state->received=true;state->ready.notify_one();
      });
  if (!subscription) throw std::runtime_error("Cannot read board GPU mounting snapshot: "+subscription.error().message);
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    state->ready.wait_for(lock,std::chrono::milliseconds(std::max<std::uint32_t>(1,std::min<std::uint32_t>(options.first_frame_timeout_ms,30000))),[&]{return state->received;});
  }
  subscription.value().stop();
  std::lock_guard<std::mutex> lock(state->mutex);
  if (!state->projection) throw std::runtime_error(state->error.empty() ? "Board GPU mounting snapshot timed out" : state->error);
  return *state->projection;
}

inline TfSetup ConfigureBoardTf(const orbvi_sdk::RobotGroundProjection& p,
                               bool publish_mounting,const std::string& depth_frame) {
  if (!orbvi_camera_contract::ValidRobotGroundProjection(p)) throw std::runtime_error("Invalid SDK robot projection");
  // Reuse the mounting frame collision checks with a temporary identity RPY.
  auto mounting=MakeRobotMounting(p.parent_frame,p.child_frame,p.xyz,{0,0,0});
  mounting.base_from_device.quaternion=p.quaternion;
  mounting.ground_plane=p.ground_plane;
  if (!depth_frame.empty() && depth_frame!=p.child_frame)
    throw std::runtime_error("ROS_BODY frame differs from SDK mounting child frame");
  return {p.child_frame,mounting,publish_mounting};
}

// Decoder delivery must belong to the validated raw frame, never a cached image.
inline bool BoardIpmDecodedFrameMatches(const orbvi_sdk::FrameDelivery& delivery) {
  const auto* image = delivery.decoded_image ? &*delivery.decoded_image :
      delivery.decoded_images.empty() ? nullptr : &delivery.decoded_images.front();
  return image && image->timestamp_ns == delivery.frame.header.timestamp_ns &&
      image->frame_id == delivery.frame.frame_id;
}

// Compare the GPU snapshot against the audited TF geometry actually used by pose.
// A legacy, stale, conflicting or reconfigured frame never becomes robot IPM by relabeling.
inline std::string CheckBoardRobotIpm(const orbvi_sdk::FrameView& frame,
                                    const ResolvedRobotTf& tf,
                                    const std::array<double,4>& plane) {
  auto result=orbvi_sdk::ReadBoardRobotProjection(frame);
  if (!result) return result.error().message;
  const auto& p=result.value();const auto& mount=tf.base_from_device;
  if (p.parent_frame!=mount.parent || p.child_frame!=mount.child || frame.calibration_hash!=tf.cameras.camera_hash)
    return "Board IPM frame/calibration differs from TF snapshot";
  for(int i=0;i<3;++i) if(std::abs(p.xyz[i]-mount.translation[i])>1e-9)
    return "Board IPM mounting translation differs from TF";
  double dot=0;for(int i=0;i<4;++i) dot+=p.quaternion[i]*mount.quaternion[i];
  if(std::abs(std::abs(dot)-1)>1e-9) return "Board IPM mounting rotation differs from TF";
  const auto normalize=[](const auto& n) {
    auto out=n;double norm=std::hypot(std::hypot(n[0],n[1]),n[2]);
    if(n[2]<0) norm=-norm;
    for(double& v:out)v/=norm;return out;
  };
  const auto a=normalize(p.ground_plane),b=normalize(plane);
  for(int i=0;i<4;++i) if(!std::isfinite(b[i]) || std::abs(a[i]-b[i])>1e-9)
    return "Board IPM ground plane differs from robot ground plane";
  return {};
}
} // namespace orbvi_ros_bridge
