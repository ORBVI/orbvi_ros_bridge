#pragma once

#include "transform_geometry.hpp"
#include <optional>
#include <map>
#include <set>

namespace orbvi_ros_bridge {

struct RobotMounting {
  TransformRecord base_from_device;
  // Optional robot-base plane: nx*x + ny*y + nz*z + d = 0 (metres).
  std::optional<std::array<double, 4>> ground_plane;
  bool pose_from_config = true;
};

// Input transforms must have passed ValidTransform. These routines preserve
// frame identities and use the same quaternion algebra for TF and IPM.
std::array<double, 3> TransformPoint(const TransformRecord &transform,
                                    const std::array<double, 3> &point);
TransformRecord InverseTransform(const TransformRecord &transform);
TransformRecord ComposeTransforms(const TransformRecord &a, const TransformRecord &b);

// ROS fixed-axis RPY: Rz(yaw) * Ry(pitch) * Rx(roll); metres and radians.
// Invalid/missing installation data is a configuration error, never identity.
RobotMounting MakeRobotMounting(const std::string &parent,
                               const std::string &child,
                               const std::array<double, 3> &xyz,
                               const std::array<double, 3> &rpy);
RobotMounting LoadRobotMounting(const std::string &filename, bool require_pose = true);

struct TfSetup {
  std::string device_frame;
  std::optional<RobotMounting> mounting;
  bool publish_mounting_tf = false;
};
// Validates mode/ownership and resolves the ROS_BODY/device frame together.
TfSetup ConfigureTf(const std::string &mode, const std::string &mounting_config,
                    bool publish_mounting_tf, const std::string &ros_body_frame);

struct RobotTransforms {
  SensorTransforms sensors;
  std::optional<TransformRecord> imu_from_base;
};

// Input is the device-mode snapshot for mounting.base_from_device.child.
// With an incomplete device/IMU anchor, retain independent valid branches and
// omit the robot dynamic correction. The original snapshot is not modified.
RobotTransforms BuildRobotTransforms(const SensorTransforms &device,
                                     const RobotMounting &mounting,
                                     bool publish_mounting_tf);

bool RobotPoseTransform(const TransformRecord &world_from_imu,
                        const TransformRecord &imu_from_base,
                        TransformRecord *world_from_base);

// Bounded ownership audit for static TF paths. @local denotes an unpublished
// SDK/config snapshot, replaced by the first actual publisher of identical data.
class StaticTfRegistry {
 public:
  void Observe(const TransformRecord &record, const std::string &authority);
  void ObserveDynamic(const std::string &child);
  bool HasChild(const std::string &child) const;
  bool TracksStatic(const std::string &child) const { return entries_.count(child); }
  bool Resolve(const std::string &parent, const std::string &child,
               TransformRecord *out, std::string *error) const;
 private:
  struct Entry { TransformRecord value; std::string authority; bool conflict = false; };
  std::map<std::string, Entry> entries_;
  std::set<std::string> dynamic_children_;
  bool overflow_ = false;
};

// Requires a Z-up native world. Sets the initial vertical ground projection
// as odom origin and initial robot heading as +X; does not rotate roll/pitch.
std::optional<TransformRecord> StartupOdomTransform(
    const TransformRecord &world_from_base, const std::array<double, 4> &base_plane,
    const std::string &odom_frame);
} // namespace orbvi_ros_bridge
