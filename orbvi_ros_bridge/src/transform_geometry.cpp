#include "transform_geometry.hpp"
#include "orbvi_sdk/client.hpp"
#include "orbvi_sdk/panorama.hpp"
#include <boost/property_tree/json_parser.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <utility>

namespace orbvi_ros_bridge {
namespace {
using Matrix = std::array<double, 9>;
using Vector = std::array<double, 3>;
struct Rigid {
  Matrix r{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  Vector t{};
};
Vector Apply(const Matrix &r, const Vector &t) {
  Vector out{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      out[i] += r[3 * i + j] * t[j];
  return out;
}
Rigid Compose(const Rigid &a, const Rigid &b) {
  Rigid out;
  out.r.fill(0);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k)
        out.r[3 * i + j] += a.r[3 * i + k] * b.r[3 * k + j];
  out.t = Apply(a.r, b.t);
  for (int i = 0; i < 3; ++i)
    out.t[i] += a.t[i];
  return out;
}
Rigid Inverse(const Rigid &a) {
  Rigid out;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      out.r[3 * i + j] = a.r[3 * j + i];
  out.t = Apply(out.r, a.t);
  for (auto &v : out.t)
    v = -v;
  return out;
}
bool Valid(const Rigid &a) {
  for (double v : a.r)
    if (!std::isfinite(v))
      return false;
  for (double v : a.t)
    if (!std::isfinite(v))
      return false;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      double dot = 0;
      for (int k = 0; k < 3; ++k)
        dot += a.r[3 * k + i] * a.r[3 * k + j];
      if (std::abs(dot - (i == j ? 1.0 : 0.0)) > 1e-4)
        return false;
    }
  const auto &r = a.r;
  const double det = r[0] * (r[4] * r[8] - r[5] * r[7]) -
                     r[1] * (r[3] * r[8] - r[5] * r[6]) +
                     r[2] * (r[3] * r[7] - r[4] * r[6]);
  return std::abs(det - 1) < 1e-4;
}
std::array<double, 4> Quaternion(const Matrix &r) {
  // Select the largest component, including 180-degree rotations.
  std::array<double, 4> q{};
  const std::array<double, 4> squared{
      {1 + r[0] - r[4] - r[8], 1 - r[0] + r[4] - r[8], 1 - r[0] - r[4] + r[8],
       1 + r[0] + r[4] + r[8]}};
  const int k = static_cast<int>(
      std::max_element(squared.begin(), squared.end()) - squared.begin());
  q[k] = 0.5 * std::sqrt(std::max(0.0, squared[k]));
  const double d = 4 * q[k];
  if (k == 3) {
    q[0] = (r[7] - r[5]) / d;
    q[1] = (r[2] - r[6]) / d;
    q[2] = (r[3] - r[1]) / d;
  }
  if (k == 0) {
    q[1] = (r[1] + r[3]) / d;
    q[2] = (r[2] + r[6]) / d;
    q[3] = (r[7] - r[5]) / d;
  }
  if (k == 1) {
    q[0] = (r[1] + r[3]) / d;
    q[2] = (r[5] + r[7]) / d;
    q[3] = (r[2] - r[6]) / d;
  }
  if (k == 2) {
    q[0] = (r[2] + r[6]) / d;
    q[1] = (r[5] + r[7]) / d;
    q[3] = (r[3] - r[1]) / d;
  }
  double norm = 0;
  for (double v : q)
    norm += v * v;
  norm = std::sqrt(norm);
  for (auto &v : q)
    v /= norm;
  return q;
}
bool FrameName(const std::string &name) {
  return !name.empty() && name.front() != '/' && name.back() != '/' &&
         name.find("//") == std::string::npos &&
         std::all_of(name.begin(), name.end(), [](unsigned char c) {
           return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '/';
         });
}
std::string JsonString(const std::string &s) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : s) {
    if (c == '"' || c == '\\')
      out << '\\' << c;
    else if (c < 32)
      out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
          << static_cast<int>(c) << std::dec;
    else
      out << c;
  }
  out << '"';
  return out.str();
}
std::string Meta(const orbvi_sdk::FrameView &frame, const char *key) {
  if (!frame.metadata)
    return {};
  const auto it = frame.metadata->find(key);
  return it == frame.metadata->end() ? "" : it->second;
}
bool UInt(const std::string &s, std::uint64_t *out) {
  if (s.empty())
    return false;
  *out = 0;
  for (unsigned char c : s) {
    if (c < '0' || c > '9' ||
        *out > (std::numeric_limits<std::uint64_t>::max() - (c - '0')) / 10)
      return false;
    *out = *out * 10 + (c - '0');
  }
  return true;
}
// The SDK's complete LiDAR parser requires all four independent FAST-Calib
// files. TF only depends on the front anchor and IMU. Decode those separately
// on failure, without manufacturing the missing files or changing the SDK.
template <std::size_t N>
std::array<double, N> JsonArray(const boost::property_tree::ptree &tree,
                                const char *key) {
  std::array<double, N> values{};
  const auto &array = tree.get_child(key);
  if (array.size() != N)
    throw std::runtime_error("invalid array size");
  std::size_t i = 0;
  for (const auto &entry : array) {
    if (!entry.first.empty() || !entry.second.empty())
      throw std::runtime_error("invalid array element");
    values[i] = entry.second.get_value<double>();
    if (!std::isfinite(values[i++]))
      throw std::runtime_error("nonfinite value");
  }
  return values;
}
std::optional<orbvi_sdk::LidarCalibration>
PartialLidar(const orbvi_sdk::CalibrationStatus &status) {
  try {
    boost::property_tree::ptree json;
    std::istringstream input(status.raw_json);
    boost::property_tree::read_json(input, json);
    orbvi_sdk::LidarCalibration out;
    out.calibration_version = json.get<std::string>("calibration_version", "");
    if (!status.calibration_version.empty() &&
        !out.calibration_version.empty() &&
        status.calibration_version != out.calibration_version)
      return {};
    out.camera_calibration_hash =
        json.get<std::string>("calibration_hash", status.calibration_hash);
    if (!status.calibration_hash.empty() &&
        !out.camera_calibration_hash.empty() &&
        status.calibration_hash != out.camera_calibration_hash)
      return {};
    const auto &meta = json.get_child("lidar_calibration");
    out.calibration_hash = meta.get<std::string>("lidar_calibration_hash");
    if (meta.get<std::string>("status") != "READY" ||
        meta.get<std::string>("lidar_model") != "MID360" ||
        out.calibration_hash.empty() ||
        meta.get<std::string>("lidar_calibration_source").empty())
      return {};
    if (const auto entries = json.get_child_optional("lidar_extrinsics")) {
      for (const auto &entry : *entries) {
        try {
          const auto &e = entry.second;
          orbvi_sdk::LidarCameraExtrinsic camera;
          camera.camera_id = e.get<std::uint32_t>("camera_id");
          camera.camera_index = e.get<std::uint32_t>("camera_index");
          const std::array<std::uint32_t, 4> ids{{3, 2, 4, 5}};
          camera.source_frame = e.get<std::string>("source_frame");
          camera.target_frame = e.get<std::string>("target_frame");
          if (camera.camera_index >= ids.size() ||
              ids[camera.camera_index] != camera.camera_id ||
              camera.source_frame != "lidar" ||
              camera.target_frame !=
                  "camera_" + std::to_string(camera.camera_id) ||
              e.get<std::string>("convention") !=
                  "p_target=R_target_from_source*p_source+t_target_from_source")
            continue;
          camera.rotation_matrix_3x3 = JsonArray<9>(e, "rotation_matrix_3x3");
          camera.translation_m = JsonArray<3>(e, "translation_m");
          out.camera_extrinsics.push_back(camera);
        } catch (const std::exception &) { /* Omit only this dependency. */
        }
      }
    }
    try {
      if (json.get<bool>("lidar_imu_extrinsics_available", false)) {
        const auto &e = json.get_child("lidar_imu_extrinsic");
        orbvi_sdk::LidarImuExtrinsic imu;
        imu.source_frame = e.get<std::string>("source_frame");
        imu.target_frame = e.get<std::string>("target_frame");
        imu.translation_unit = e.get<std::string>("translation_unit");
        if (e.get<int>("schema_version") == 1 &&
            e.get<std::string>("sensor_model") == "MID360" &&
            e.get<std::string>("parameter_source") == "manufacturer_nominal" &&
            imu.source_frame == "mid360_lidar" &&
            imu.target_frame == "mid360_imu" && imu.translation_unit == "m") {
          imu.rotation_matrix = JsonArray<9>(e, "rotation_matrix");
          imu.translation_m = JsonArray<3>(e, "translation_m");
          if (Valid({imu.rotation_matrix, imu.translation_m}))
            out.imu_extrinsic = imu;
        }
      }
    } catch (const std::exception &) { /* Camera rig remains independent. */
    }
    return out;
  } catch (const std::exception &) {
    return {};
  }
}
} // namespace

std::string RawCameraFrameId(const std::string &id) {
  std::uint64_t value;
  return UInt(id, &value) ? "orbvi/camera/" + std::to_string(value) : "";
}
bool ValidTransform(const TransformRecord &t) {
  if (!FrameName(t.parent) || !FrameName(t.child) || t.parent == t.child)
    return false;
  for (double v : t.translation)
    if (!std::isfinite(v))
      return false;
  double norm = 0;
  for (double v : t.quaternion) {
    if (!std::isfinite(v))
      return false;
    norm += v * v;
  }
  return std::isfinite(norm) && std::abs(norm - 1) < 1e-6;
}
SensorTransforms
BuildSensorTransforms(const orbvi_sdk::LidarCalibration *lidar,
                      const orbvi_sdk::PanoramaCalibration *rig,
                      const std::string &depth_frame) {
  SensorTransforms out;
  if (lidar) {
    out.lidar_hash = lidar->calibration_hash;
    out.camera_hash = lidar->camera_calibration_hash;
  }
  if (rig) {
    out.camera_hash = rig->calibration_hash;
    if (rig->cameras.size() != 4)
      out.diagnostics.push_back("incomplete_camera_rig");
  }
  std::map<std::string, std::string> parents;
  const auto add = [&](const std::string &parent, const std::string &child,
                       const Rigid &value) {
    if (!Valid(value)) {
      out.diagnostics.push_back("invalid_rigid_transform:" + child);
      return false;
    }
    TransformRecord record{parent, child, value.t, Quaternion(value.r)};
    if (!ValidTransform(record) || parents.count(child)) {
      out.diagnostics.push_back("invalid_or_duplicate_frame:" + child);
      return false;
    }
    std::string ancestor = parent;
    while (parents.count(ancestor)) {
      ancestor = parents.at(ancestor);
      if (ancestor == child)
        break;
    }
    if (ancestor == child) {
      out.diagnostics.push_back("frame_cycle:" + child);
      return false;
    }
    parents[child] = parent;
    out.transforms.push_back(std::move(record));
    return true;
  };
  bool has_imu = false;
  Rigid imu_from_lidar;
  if (lidar && lidar->imu_extrinsic) {
    const auto &imu = *lidar->imu_extrinsic;
    imu_from_lidar = {imu.rotation_matrix, imu.translation_m};
    has_imu = imu.source_frame == "mid360_lidar" &&
              imu.target_frame == "mid360_imu" && imu.translation_unit == "m" &&
              Valid(imu_from_lidar);
  }
  if (has_imu) {
    add("aft_mapped", "mid360_imu",
        Rigid{}); // Same physical IMU as algorithm state.
    add("aft_mapped", "livox_frame", imu_from_lidar);
  } else
    out.diagnostics.push_back("missing_or_invalid_lidar_imu_extrinsic");

  if (!FrameName(depth_frame) || depth_frame == "aft_mapped" ||
      depth_frame == "livox_frame" || depth_frame == "mid360_imu" ||
      depth_frame == "camera_init" ||
      depth_frame.rfind("orbvi/camera/", 0) == 0) {
    out.diagnostics.push_back("invalid_depth_frame");
    return out;
  }
  // ROS_BODY maps reference optical axes (x,y,z) to (z,-x,-y).
  const Rigid base_from_reference{{{0, 0, 1, -1, 0, 0, 0, -1, 0}}, {}};
  bool has_base = false;
  const bool compatible_hashes =
      !lidar || !rig || lidar->camera_calibration_hash.empty() ||
      rig->calibration_hash.empty() ||
      lidar->camera_calibration_hash == rig->calibration_hash;
  if (!compatible_hashes)
    out.diagnostics.push_back("camera_calibration_hash_mismatch");
  if (has_imu && rig && compatible_hashes) {
    const orbvi_sdk::PanoramaCameraCalibration *front = nullptr;
    for (const auto &c : rig->cameras)
      if (c.intrinsics.role == "front") {
        if (front) {
          out.diagnostics.push_back("duplicate_front_camera");
          return out;
        }
        front = &c;
      }
    if (front && front->has_full_extrinsics) {
      const Rigid reference_from_front{front->rotation_cam_to_reference,
                                       front->translation_cam_in_reference_m};
      const orbvi_sdk::LidarCameraExtrinsic *anchor = nullptr;
      for (const auto &e : lidar->camera_extrinsics)
        if (e.camera_id == front->intrinsics.camera_id) {
          if (anchor) {
            out.diagnostics.push_back("duplicate_front_anchor");
            return out;
          }
          anchor = &e;
        }
      if (anchor) {
        const Rigid front_from_lidar{anchor->rotation_matrix_3x3,
                                     anchor->translation_m};
        if (anchor->source_frame == "lidar" &&
            anchor->target_frame ==
                "camera_" + std::to_string(front->intrinsics.camera_id) &&
            Valid(reference_from_front) && Valid(front_from_lidar)) {
          const auto base_from_lidar =
              Compose(Compose(base_from_reference, reference_from_front),
                      front_from_lidar);
          has_base = add("aft_mapped", depth_frame,
                         Compose(imu_from_lidar, Inverse(base_from_lidar)));
        }
      }
    }
  }
  if (!has_base)
    out.diagnostics.push_back("missing_or_invalid_front_anchor_or_rig");
  if (rig) {
    std::set<std::uint32_t> ids;
    std::set<std::string> roles;
    for (const auto &c : rig->cameras) {
      if (!ids.insert(c.intrinsics.camera_id).second ||
          !roles.insert(c.intrinsics.role).second) {
        out.diagnostics.push_back("duplicate_camera_identity");
        return out;
      }
    }
    for (const auto &c : rig->cameras) {
      const Rigid reference_from_camera{c.rotation_cam_to_reference,
                                        c.translation_cam_in_reference_m};
      if (!c.has_full_extrinsics || !Valid(reference_from_camera)) {
        out.diagnostics.push_back("invalid_camera:" +
                                  std::to_string(c.intrinsics.camera_id));
        continue;
      }
      // This branch is meaningful even without a LiDAR/IMU anchor.
      add(depth_frame, RawCameraFrameId(std::to_string(c.intrinsics.camera_id)),
          Compose(base_from_reference, reference_from_camera));
    }
  } else
    out.diagnostics.push_back("missing_camera_rig");
  return out;
}

SensorTransforms
LoadSensorTransforms(const orbvi_sdk::CalibrationStatus &calibration,
                     const std::string &depth_frame) {
  const auto lidar = orbvi_sdk::MakeLidarCalibration(calibration);
  const auto rig = orbvi_sdk::MakePanoramaCalibration(calibration);
  const auto partial = lidar ? std::optional<orbvi_sdk::LidarCalibration>{}
                             : PartialLidar(calibration);
  auto out = BuildSensorTransforms(lidar ? &lidar.value()
                                         : (partial ? &*partial : nullptr),
                                   rig ? &rig.value() : nullptr, depth_frame);
  if (!lidar)
    out.diagnostics.push_back("lidar_calibration:" + lidar.error().message);
  if (!rig)
    out.diagnostics.push_back("camera_calibration:" + rig.error().message);
  return out;
}

std::string ExtrinsicsJson(const SensorTransforms &snapshot) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17);
  out << "{\n  \"schema_version\":1,\n  "
         "\"convention\":\"p_parent=R_parent_from_child*p_child+t_parent_from_"
         "child\",\n  \"translation_unit\":\"m\",\n  "
         "\"quaternion_order\":\"xyzw\",\n  \"camera_calibration_hash\":"
      << JsonString(snapshot.camera_hash)
      << ",\n  \"lidar_calibration_hash\":" << JsonString(snapshot.lidar_hash)
      << ",\n  \"transforms\":[";
  bool first = true;
  for (const auto &t : snapshot.transforms) {
    if (!first)
      out << ',';
    first = false;
    out << "\n    {\"parent\":" << JsonString(t.parent)
        << ",\"child\":" << JsonString(t.child) << ",\"translation_m\":[";
    for (int i = 0; i < 3; ++i) {
      if (i)
        out << ',';
      out << t.translation[i];
    }
    out << "],\"quaternion_xyzw\":[";
    for (int i = 0; i < 4; ++i) {
      if (i)
        out << ',';
      out << t.quaternion[i];
    }
    out << "]}";
  }
  out << "\n  ],\n  \"diagnostics\":[";
  first = true;
  for (const auto &d : snapshot.diagnostics) {
    if (!first)
      out << ',';
    first = false;
    out << JsonString(d);
  }
  out << "]\n}\n";
  return out.str();
}

bool PoseTransformGate::Accept(const orbvi_sdk::FrameView &frame,
                               bool *changed) {
  if (changed)
    *changed = false;
  if (frame.timestamp_ns == 0 ||
      frame.timestamp_domain != orbvi_sdk::TimestampDomain::Realtime ||
      Meta(frame, "timestamp_valid") == "false" ||
      (!Meta(frame, "tracking_state").empty() &&
       Meta(frame, "tracking_state") != "TRACKING"))
    return false;
  const auto boot = Meta(frame, "boot_id"),
             generation = Meta(frame, "generation");
  const std::string epoch =
      boot.empty() && generation.empty() ? "" : boot + ":" + generation;
  const bool new_epoch = !epoch.empty() && epoch != epoch_;
  std::uint64_t sequence = 0;
  const bool has_sequence = UInt(Meta(frame, "sequence"), &sequence);
  if (new_epoch) {
    if (std::find(retired_epochs_.begin(), retired_epochs_.end(), epoch) !=
        retired_epochs_.end())
      return false;
  } else if (frame.timestamp_ns <= timestamp_ ||
             (has_sequence && timestamp_ != 0 &&
              (sequence == sequence_ ||
               (!epoch.empty() && sequence < sequence_))))
    return false;
  if (new_epoch) {
    if (!epoch_.empty()) {
      if (retired_epochs_.size() == 16)
        retired_epochs_.erase(retired_epochs_.begin());
      retired_epochs_.push_back(epoch_);
    }
    if (changed)
      *changed = timestamp_ != 0;
    epoch_ = epoch;
  } else if (has_sequence && sequence < sequence_ && changed)
    *changed = true;
  timestamp_ = frame.timestamp_ns;
  sequence_ = sequence;
  return true;
}
} // namespace orbvi_ros_bridge
