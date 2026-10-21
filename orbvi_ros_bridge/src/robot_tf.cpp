#include "robot_tf.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <deque>
#include <sstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace orbvi_ros_bridge {
namespace {
using Q = std::array<double, 4>;
using V = std::array<double, 3>;
Q Multiply(const Q &a, const Q &b) {
  Q q{{a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
       a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
       a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
       a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]}};
  const double n = std::hypot(std::hypot(q[0], q[1]), std::hypot(q[2], q[3]));
  for (auto &v : q) v /= n;
  return q;
}
V Rotate(const Q &q, const V &v) {
  const V cross{{q[1]*v[2]-q[2]*v[1], q[2]*v[0]-q[0]*v[2],
                 q[0]*v[1]-q[1]*v[0]}};
  const V second{{q[1]*cross[2]-q[2]*cross[1], q[2]*cross[0]-q[0]*cross[2],
                  q[0]*cross[1]-q[1]*cross[0]}};
  V out{};
  for (int i = 0; i < 3; ++i) out[i] = v[i]+2*(q[3]*cross[i]+second[i]);
  return out;
}
TransformRecord Inverse(const TransformRecord &a) {
  TransformRecord out{a.child, a.parent, {}, {-a.quaternion[0], -a.quaternion[1],
                                            -a.quaternion[2], a.quaternion[3]}};
  out.translation = Rotate(out.quaternion, a.translation);
  for (auto &v : out.translation) v = -v;
  return out;
}
TransformRecord Compose(const TransformRecord &a, const TransformRecord &b) {
  TransformRecord out{a.parent, b.child, Rotate(a.quaternion, b.translation),
                      Multiply(a.quaternion, b.quaternion)};
  for (int i = 0; i < 3; ++i) out.translation[i] += a.translation[i];
  return out;
}
bool Reserved(const std::string &s) {
  return s == "camera_init" || s == "aft_mapped" || s == "mid360_imu" ||
         s == "livox_frame" || s.rfind("orbvi/camera/", 0) == 0;
}
void Validate(const RobotMounting &mounting) {
  const auto &t = mounting.base_from_device;
  if (!ValidTransform(t) || Reserved(t.parent) || Reserved(t.child))
    throw std::runtime_error("Invalid or colliding robot mounting frames/transform");
}
void Keys(const YAML::Node &node, const std::set<std::string> &required,
          const std::set<std::string> &optional = {}) {
  if (!node.IsMap()) throw std::runtime_error("Mounting configuration must be a map");
  std::set<std::string> seen;
  for (const auto &entry : node) {
    const auto key = entry.first.as<std::string>();
    if ((!required.count(key) && !optional.count(key)) || !seen.insert(key).second)
      throw std::runtime_error("Unknown or duplicate mounting key: " + key);
  }
  for (const auto &key : required)
    if (!seen.count(key)) throw std::runtime_error("Missing required mounting field: " + key);
}
V Array(const YAML::Node &node) {
  if (!node.IsSequence() || node.size() != 3)
    throw std::runtime_error("Mounting xyz and rpy must each contain three numbers");
  return {{node[0].as<double>(), node[1].as<double>(), node[2].as<double>()}};
}
} // namespace

V TransformPoint(const TransformRecord &transform, const V &point) {
  auto out = Rotate(transform.quaternion, point);
  for (int i = 0; i < 3; ++i) out[i] += transform.translation[i];
  return out;
}
TransformRecord InverseTransform(const TransformRecord &transform) { return Inverse(transform); }
TransformRecord ComposeTransforms(const TransformRecord &a, const TransformRecord &b) { return Compose(a, b); }

RobotMounting MakeRobotMounting(const std::string &parent, const std::string &child,
                               const V &xyz, const V &rpy) {
  for (double v : rpy)
    if (!std::isfinite(v)) throw std::runtime_error("Nonfinite mounting RPY");
  const double cr=std::cos(rpy[0]/2), sr=std::sin(rpy[0]/2);
  const double cp=std::cos(rpy[1]/2), sp=std::sin(rpy[1]/2);
  const double cy=std::cos(rpy[2]/2), sy=std::sin(rpy[2]/2);
  RobotMounting out{{parent, child, xyz,
                    {sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                     cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy}}};
  Validate(out);
  return out;
}

RobotMounting LoadRobotMounting(const std::string &filename, bool require_pose) {
  std::ifstream input(filename, std::ios::binary);
  if (!input) throw std::runtime_error("Cannot open mounting_config: " + filename);
  // Installation files are tiny; bound accidental reads of unrelated files.
  std::string text(65537, '\0');
  input.read(&text[0], static_cast<std::streamsize>(text.size()));
  const auto size = input.gcount();
  if (input.bad() || size > 65536)
    throw std::runtime_error("Cannot read mounting_config or file exceeds 64 KiB");
  text.resize(static_cast<std::size_t>(size));
  const auto root = YAML::Load(text);
  Keys(root, {"orbvi"});
  Keys(root["orbvi"], {"mounting"}, {"ground_plane"});
  const auto mount = root["orbvi"]["mounting"];
  if (require_pose) Keys(mount, {"parent_frame", "child_frame", "xyz", "rpy"});
  else Keys(mount, {"parent_frame", "child_frame"});
  auto out = MakeRobotMounting(mount["parent_frame"].as<std::string>(),
                          mount["child_frame"].as<std::string>(),
                          require_pose ? Array(mount["xyz"]) : V{},
                          require_pose ? Array(mount["rpy"]) : V{});
  out.pose_from_config = require_pose;
  if (const auto plane = root["orbvi"]["ground_plane"]) {
    if (!plane.IsSequence() || plane.size() != 4)
      throw std::runtime_error("ground_plane must be [nx, ny, nz, d]");
    std::array<double, 4> values{};
    for (std::size_t i = 0; i < 4; ++i) {
      values[i] = plane[i].as<double>();
      if (!std::isfinite(values[i])) throw std::runtime_error("Nonfinite ground_plane");
    }
    if (std::hypot(std::hypot(values[0], values[1]), values[2]) < 1e-9)
      throw std::runtime_error("ground_plane has a zero normal");
    out.ground_plane = values;
  }
  return out;
}

TfSetup ConfigureTf(const std::string &mode, const std::string &mounting_config,
                    bool publish_mounting_tf, const std::string &ros_body_frame) {
  TfSetup out;
  out.device_frame = ros_body_frame.empty() ? kDefaultDeviceFrame : ros_body_frame;
  if (mode == "robot") {
    if (mounting_config.empty())
      throw std::runtime_error("tf_mode=robot requires mounting_config");
    out.mounting = LoadRobotMounting(mounting_config, publish_mounting_tf);
    out.publish_mounting_tf = publish_mounting_tf;
    const auto &child = out.mounting->base_from_device.child;
    if (!ros_body_frame.empty() && ros_body_frame != child)
      throw std::runtime_error("ROS_BODY depth_pointcloud_frame_id must match mounting.child_frame");
    out.device_frame = child;
  } else if (mode != "device") {
    throw std::runtime_error("tf_mode must be device or robot");
  } else if (!mounting_config.empty() || publish_mounting_tf) {
    throw std::runtime_error("mounting_config/publish_mounting_tf requires tf_mode=robot");
  }
  return out;
}

RobotTransforms BuildRobotTransforms(const SensorTransforms &device,
                                     const RobotMounting &mounting,
                                     bool publish_mounting_tf) {
  Validate(mounting);
  RobotTransforms out;
  out.sensors = device;
  out.sensors.transforms.clear();
  const auto &mount = mounting.base_from_device;
  const auto anchor = std::find_if(device.transforms.begin(), device.transforms.end(),
      [&](const auto &t) { return t.parent == "aft_mapped" && t.child == mount.child; });
  const bool has_anchor = anchor != device.transforms.end() && ValidTransform(*anchor);
  if (has_anchor && mounting.pose_from_config) {
    const auto correction = Compose(*anchor, Inverse(mount));
    if (ValidTransform(correction)) out.imu_from_base = correction;
    else out.sensors.diagnostics.push_back("robot_tf_invalid_composed_robot_pose");
  } else if (!has_anchor)
    out.sensors.diagnostics.push_back("robot_tf_missing_device_imu_anchor");
  for (const auto &t : device.transforms) {
    if (!ValidTransform(t) || t.child == mount.parent)
      throw std::runtime_error("Invalid or colliding sensor/mounting tree");
    if (t.parent == mount.child) out.sensors.transforms.push_back(t);
    else if (!has_anchor && t.parent == "aft_mapped") {
      // The independent IMU/LiDAR calibration remains useful even when the
      // camera-to-LiDAR anchor cannot connect it to the device/robot branch.
      out.sensors.transforms.push_back(t);
    }
    else if (has_anchor && t.parent == "aft_mapped" && t.child != mount.child) {
      const auto transformed = Compose(Inverse(*anchor), t);
      if (ValidTransform(transformed)) out.sensors.transforms.push_back(transformed);
      else out.sensors.diagnostics.push_back("robot_tf_invalid_composed_sensor:" + t.child);
      // Preserve the IMU Odometry child as an alias, without its old dynamic edge.
      if (t.child == "mid360_imu") out.sensors.transforms.push_back(Inverse(t));
    }
  }
  if (publish_mounting_tf) {
    if (!mounting.pose_from_config) throw std::runtime_error("URDF mounting has no configured pose");
    out.sensors.transforms.push_back(mount);
  }
  return out;
}

bool RobotPoseTransform(const TransformRecord &world_from_imu,
                        const TransformRecord &imu_from_base,
                        TransformRecord *world_from_base) {
  if (!world_from_base || !ValidTransform(world_from_imu) ||
      !ValidTransform(imu_from_base) || world_from_imu.parent != "camera_init" ||
      world_from_imu.child != "aft_mapped" || imu_from_base.parent != "aft_mapped")
    return false;
  const auto result = Compose(world_from_imu, imu_from_base);
  if (!ValidTransform(result)) return false;
  *world_from_base = result;
  return true;
}
void StaticTfRegistry::Observe(const TransformRecord &record, const std::string &authority) {
  if (entries_.size() >= 4096 && !entries_.count(record.child)) { overflow_ = true; return; }
  auto found = entries_.find(record.child);
  if (found == entries_.end()) {
    entries_.emplace(record.child, Entry{record, authority, !ValidTransform(record)});
    return;
  }
  auto &e = found->second;
  bool same = ValidTransform(record) && e.value.parent == record.parent;
  for (int i=0;i<3;++i) same = same && std::abs(e.value.translation[i]-record.translation[i]) < 1e-10;
  double dot=0; for (int i=0;i<4;++i) dot += e.value.quaternion[i]*record.quaternion[i];
  same = same && std::abs(std::abs(dot)-1) < 1e-10;
  if (!same || (e.authority != "@local" && authority != "@local" && e.authority != authority))
    e.conflict = true;
  if (e.authority == "@local" && authority != "@local") e.authority = authority;
}
void StaticTfRegistry::ObserveDynamic(const std::string &child) {
  if (dynamic_children_.size() >= 4096) overflow_ = true;
  else dynamic_children_.insert(child);
}
bool StaticTfRegistry::HasChild(const std::string &child) const { return entries_.count(child) || dynamic_children_.count(child); }
bool StaticTfRegistry::Resolve(const std::string &parent, const std::string &child,
                               TransformRecord *out, std::string *error) const {
  const auto fail = [&](const std::string &reason) { if (error) *error=reason; return false; };
  if (overflow_) return fail("static TF audit exceeds 4096 frames");
  struct Step { std::string frame; TransformRecord pose; std::vector<std::string> edges; };
  std::deque<Step> queue{{parent, {parent,parent,{}, {0,0,0,1}}, {}}};
  std::set<std::string> seen{parent};
  while (!queue.empty()) {
    auto step=std::move(queue.front());queue.pop_front();
    if (step.frame == child) {
      for (const auto &edge : step.edges) {
        const auto &e=entries_.at(edge);
        if (e.conflict || dynamic_children_.count(edge)) return fail("conflicting/changing/nonstatic TF child: "+edge);
        std::set<std::string> ancestors;
        auto frame=edge;
        while(entries_.count(frame)) {
          if (!ancestors.insert(frame).second) return fail("static TF cycle: "+frame);
          frame=entries_.at(frame).value.parent;
        }
      }
      if (out) *out=step.pose;
      return true;
    }
    for (const auto &[key,e] : entries_) {
      const auto &t=e.value;
      if (!ValidTransform(t)) continue;
      const bool forward=t.parent==step.frame, backward=t.child==step.frame;
      if (!forward && !backward) continue;
      const auto next=forward?t.child:t.parent;
      if (!seen.insert(next).second) continue;
      auto edges=step.edges;edges.push_back(key);
      queue.push_back({next, Compose(step.pose,forward?t:Inverse(t)),std::move(edges)});
    }
  }
  return fail("waiting for static TF: "+parent+" <- "+child);
}
std::optional<TransformRecord> StartupOdomTransform(
    const TransformRecord &pose, const std::array<double,4> &plane, const std::string &odom) {
  if (!ValidTransform(pose) || Reserved(odom) || odom==pose.child || odom==pose.parent ||
      !ValidTransform({odom,pose.parent,{}, {0,0,0,1}})) return {};
  for (auto v:plane) if (!std::isfinite(v)) return {};
  const double normal=std::hypot(std::hypot(plane[0],plane[1]),plane[2]);
  if (!std::isfinite(normal) || normal < 1e-9) return {};
  const auto vertical=Rotate(Inverse(pose).quaternion,{0,0,1});
  double denominator=0;for(int i=0;i<3;++i) denominator+=plane[i]/normal*vertical[i];
  if (std::abs(denominator)<0.1) return {};
  const auto heading=Rotate(pose.quaternion,{1,0,0});
  if (std::hypot(heading[0],heading[1])<1e-6) return {};
  const double yaw=std::atan2(heading[1],heading[0]);
  const double height=-plane[3]/normal/denominator;
  const auto origin=TransformPoint(pose,{height*vertical[0],height*vertical[1],height*vertical[2]});
  TransformRecord result{odom,pose.parent,{}, {0,0,-std::sin(yaw/2),std::cos(yaw/2)}};
  result.translation=Rotate(result.quaternion,origin);
  for(auto &v:result.translation) v=-v;
  if (!ValidTransform(result)) return {};
  return result;
}
} // namespace orbvi_ros_bridge
