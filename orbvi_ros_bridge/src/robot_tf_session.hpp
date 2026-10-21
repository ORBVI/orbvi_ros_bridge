#pragma once
#include "robot_tf.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace orbvi_ros_bridge {
struct ResolvedRobotTf {
  TransformRecord base_from_device;
  std::optional<TransformRecord> imu_from_base;
  SensorTransforms cameras;
};

// Shared ROS1/ROS2 policy. Geometry is obtained with the supplied tf2 lookup;
// the registry independently audits that every traversed edge is static and has
// one publisher. Installation and sensor snapshots never silently hot-reload.
class RobotTfSession {
 public:
  using Lookup = std::function<TransformRecord(const std::string &,const std::string &)>;
  using Install = std::function<void(const TransformRecord &,bool)>;
  RobotTfSession(TfSetup setup, SensorTransforms sensors, bool publish_sensors,
                 bool publish_odom, std::string odom_frame, Lookup lookup, Install install,
                 std::chrono::steady_clock::time_point started=std::chrono::steady_clock::now())
      : setup_(std::move(setup)), sensors_(std::move(sensors)), publish_sensors_(publish_sensors),
        publish_odom_(publish_odom), odom_frame_(std::move(odom_frame)),
        lookup_(std::move(lookup)), install_(std::move(install)), started_(started) {
    for (const auto &t:sensors_.transforms) Seed(t,false);
    if (setup_.publish_mounting_tf && !publish_sensors_) {
      Seed(setup_.mounting->base_from_device,false);
      mounting_installed_=true;
    }
  }
  bool Observe(const TransformRecord &t,const std::string &owner,bool is_static) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_static) registry_.Observe(t,owner);
    else registry_.ObserveDynamic(t.child);
    if (snapshot_ && !AuditSnapshot()) snapshot_.reset();
    return is_static && registry_.TracksStatic(t.child);
  }
  void Poll() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (blocked_) return;
    const auto &mount=setup_.mounting->base_from_device;
    if (setup_.publish_mounting_tf && !mounting_installed_) {
      // Allow discovery of existing static publishers before claiming the edge.
      if (std::chrono::steady_clock::now()-started_<std::chrono::milliseconds(500)) {
        error_="discovering installation TF publishers";return;
      }
      if (registry_.HasChild(mount.child)) {
        blocked_=true;error_="installation TF already has an external publisher: "+mount.child;
        snapshot_.reset();return;
      }
      Seed(mount,true);mounting_installed_=true;
    }
    auto next=std::make_shared<ResolvedRobotTf>();
    std::string imu_error="missing static mid360_imu branch";
    try {
      next->base_from_device=Query(mount.parent,mount.child);
      if (registry_.HasChild("mid360_imu")) {
        try {
          auto t=InverseTransform(Query(mount.parent,"mid360_imu"));
          t.parent="aft_mapped";t.child=mount.parent;
          next->imu_from_base=t;
          imu_error.clear();
        } catch(const std::exception &e) { imu_error=e.what(); }
      }
      for (const auto &t:sensors_.transforms) {
        if (t.parent==mount.child && t.child.rfind("orbvi/camera/",0)==0)
          next->cameras.transforms.push_back(Query(mount.parent,t.child));
      }
      next->cameras.camera_hash=sensors_.camera_hash;
      if (snapshot_) {
        if (!AuditSnapshot()) { snapshot_.reset();return; }
      }
      // A previously independent IMU branch can become connected later. Keep
      // the audited camera geometry and adopt newly available dependencies.
      snapshot_=std::move(next);
      error_=std::move(imu_error);
    } catch (const std::exception &e) { error_=e.what();snapshot_.reset(); }
  }
  std::shared_ptr<const ResolvedRobotTf> Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);return snapshot_;
  }
  std::string Error() const { std::lock_guard<std::mutex> lock(mutex_);return error_; }
  bool Pose(const TransformRecord &source,bool producer_changed,TransformRecord *result) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (blocked_ || !snapshot_ || !snapshot_->imu_from_base || !AuditSnapshot()) return false;
    if (!RobotPoseTransform(source,*snapshot_->imu_from_base,result)) return false;
    if (!publish_odom_) return true;
    if (producer_changed && odom_) {
      blocked_=true;error_="algorithm restarted with startup odom enabled; restart bridge to define the new world";
      snapshot_.reset();return false;
    }
    if (!setup_.mounting->ground_plane) return false;
    if (!odom_) {
      if (registry_.HasChild(source.parent)) {
        blocked_=true;error_="camera_init already has an external parent/publisher";return false;
      }
      TransformRecord existing;
      if (registry_.Resolve(result->child,odom_frame_,&existing,nullptr)) {
        blocked_=true;error_="odom frame already belongs to the robot static subtree";
        snapshot_.reset();return false;
      }
      odom_=StartupOdomTransform(*result,*setup_.mounting->ground_plane,odom_frame_);
      if (!odom_) { error_="cannot initialize odom from robot heading/ground plane";return false; }
      Seed(*odom_,true);
    } else {
      try { Query(odom_->parent,odom_->child); }
      catch(const std::exception &e) { error_=e.what();blocked_=true;snapshot_.reset();return false; }
    }
    return true;
  }
 private:
  void Seed(const TransformRecord &t,bool broadcast) {
    registry_.Observe(t,"@local");install_(t,broadcast);
  }
  TransformRecord Query(const std::string &parent,const std::string &child) const {
    TransformRecord audited;std::string error;
    if (!registry_.Resolve(parent,child,&audited,&error)) throw std::runtime_error(error);
    auto actual=lookup_(parent,child);
    if (!ValidTransform(actual)) throw std::runtime_error("invalid tf2 result: "+child);
    // Both consumers use the tf2 result; reject inconsistencies with its static path.
    for(int i=0;i<3;++i) if(std::abs(actual.translation[i]-audited.translation[i])>1e-9)
      throw std::runtime_error("tf2/static audit mismatch: "+child);
    double dot=0;for(int i=0;i<4;++i) dot+=actual.quaternion[i]*audited.quaternion[i];
    if(std::abs(std::abs(dot)-1)>1e-9) throw std::runtime_error("tf2/static rotation mismatch: "+child);
    return actual;
  }
  bool AuditSnapshot() {
    try {
      const auto &m=setup_.mounting->base_from_device;
      Query(m.parent,m.child);
      if(snapshot_->imu_from_base) Query(m.parent,"mid360_imu");
      for(const auto &t:snapshot_->cameras.transforms) Query(t.parent,t.child);
      return true;
    } catch(const std::exception &e) { error_=e.what();return false; }
  }
  TfSetup setup_;
  SensorTransforms sensors_;
  bool publish_sensors_,publish_odom_,mounting_installed_=false,blocked_=false;
  std::string odom_frame_,error_;
  Lookup lookup_;Install install_;
  std::chrono::steady_clock::time_point started_;
  mutable std::mutex mutex_;
  StaticTfRegistry registry_;
  std::shared_ptr<const ResolvedRobotTf> snapshot_;
  std::optional<TransformRecord> odom_;
};
} // namespace orbvi_ros_bridge
