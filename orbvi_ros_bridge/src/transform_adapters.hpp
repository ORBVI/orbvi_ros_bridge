#pragma once
#include "transform_geometry.hpp"
#include "robot_tf.hpp"
#include <cmath>

namespace orbvi_ros_bridge {
template <class Message>
TransformRecord FromTransformMessage(const Message &message) {
  const auto &p=message.transform.translation;
  const auto &q=message.transform.rotation;
  const double norm=std::hypot(std::hypot(q.x,q.y),std::hypot(q.z,q.w));
  return {message.header.frame_id,message.child_frame_id,{p.x,p.y,p.z},
          {q.x/norm,q.y/norm,q.z/norm,q.w/norm}};
}
// ROS1 and ROS2 expose the same fields; keep only serialization here.
template <class Message>
Message ToTransformMessage(const TransformRecord &record) {
  Message out;
  out.header.frame_id = record.parent;
  out.child_frame_id = record.child;
  out.transform.translation.x = record.translation[0];
  out.transform.translation.y = record.translation[1];
  out.transform.translation.z = record.translation[2];
  out.transform.rotation.x = record.quaternion[0];
  out.transform.rotation.y = record.quaternion[1];
  out.transform.rotation.z = record.quaternion[2];
  out.transform.rotation.w = record.quaternion[3];
  return out;
}
template <class Odometry, class Message>
bool OdometryTransform(const Odometry &odometry, Message *out,
                       const TransformRecord *imu_from_base = nullptr) {
  if (!out)
    return false;
  const auto &p = odometry.pose.pose.position;
  const auto &q = odometry.pose.pose.orientation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(norm) || norm < 1e-12)
    return false;
  TransformRecord record{
      odometry.header.frame_id,
      odometry.child_frame_id,
      {p.x, p.y, p.z},
      {q.x / norm, q.y / norm, q.z / norm, q.w / norm}};
  if (!ValidTransform(record))
    return false;
  if (imu_from_base && !RobotPoseTransform(record, *imu_from_base, &record))
    return false;
  *out = ToTransformMessage<Message>(record);
  out->header = odometry.header;
  return true;
}
} // namespace orbvi_ros_bridge
