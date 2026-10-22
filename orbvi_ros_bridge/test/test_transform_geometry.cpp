#include "orbvi_sdk/client.hpp"
#include "transform_geometry.hpp"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <set>

using namespace orbvi_ros_bridge;
namespace {
struct Fixture {
  orbvi_sdk::LidarCalibration lidar;
  orbvi_sdk::PanoramaCalibration rig;
  Fixture() {
    lidar.calibration_hash = "lidar-hash";
    rig.calibration_hash = "camera-hash";
    orbvi_sdk::LidarImuExtrinsic imu;
    imu.source_frame = "mid360_lidar";
    imu.target_frame = "mid360_imu";
    imu.translation_unit = "m";
    imu.rotation_matrix = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    imu.translation_m = {-.01, -.02, .04};
    lidar.imu_extrinsic = imu;
    orbvi_sdk::LidarCameraExtrinsic anchor;
    anchor.camera_id = 3;
    anchor.source_frame = "lidar";
    anchor.target_frame = "camera_3";
    anchor.translation_m = {4, 5, 6};
    lidar.camera_extrinsics.push_back(anchor);
    const std::array<unsigned, 4> ids{{3, 2, 4, 5}};
    const std::array<std::string, 4> roles{{"front", "right", "rear", "left"}};
    for (int i = 0; i < 4; ++i) {
      orbvi_sdk::PanoramaCameraCalibration c;
      c.intrinsics.camera_id = ids[i];
      c.intrinsics.role = roles[i];
      c.has_full_extrinsics = true;
      c.rotation_cam_to_reference = {0, -1, 0, 1, 0, 0, 0, 0, 1};
      c.translation_cam_in_reference_m = {1. + i, 2, 3};
      rig.cameras.push_back(c);
    }
  }
};
const TransformRecord &Find(const SensorTransforms &s,
                            const std::string &child) {
  const auto it = std::find_if(s.transforms.begin(), s.transforms.end(),
                               [&](const auto &t) { return t.child == child; });
  if (it == s.transforms.end())
    throw std::runtime_error("Missing transform: " + child);
  return *it;
}
std::array<double, 3> Point(const TransformRecord &t, std::array<double, 3> p) {
  const auto &q = t.quaternion;
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  return {{(1 - 2 * y * y - 2 * z * z) * p[0] + 2 * (x * y - z * w) * p[1] +
               2 * (x * z + y * w) * p[2] + t.translation[0],
           2 * (x * y + z * w) * p[0] + (1 - 2 * x * x - 2 * z * z) * p[1] +
               2 * (y * z - x * w) * p[2] + t.translation[1],
           2 * (x * z - y * w) * p[0] + 2 * (y * z + x * w) * p[1] +
               (1 - 2 * x * x - 2 * y * y) * p[2] + t.translation[2]}};
}
} // namespace
TEST(TransformGeometry, PhysicalPointAndCameraPathsAgree) {
  Fixture f;
  const auto s = BuildSensorTransforms(&f.lidar, &f.rig);
  ASSERT_TRUE(s.diagnostics.empty());
  ASSERT_EQ(s.transforms.size(), 7u);
  const auto imu = Point(Find(s, "livox_frame"), {2, 3, 4});
  const auto base = Point(Find(s, "orbvi_link"), {13, 7, -8});
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(imu[i], base[i], 1e-12);
  EXPECT_NEAR(imu[0], -3.01, 1e-12);
  EXPECT_NEAR(imu[1], 1.98, 1e-12);
  EXPECT_NEAR(imu[2], 4.04, 1e-12);
  // The front camera point is (6,8,10). Its base and IMU path must agree.
  const auto camera_base = Point(Find(s, RawCameraFrameId("3")), {6, 8, 10});
  const auto camera_imu = Point(Find(s, "orbvi_link"), camera_base);
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(imu[i], camera_imu[i], 1e-12);
  EXPECT_EQ(Find(s, "mid360_imu").parent, "aft_mapped");
  std::map<std::string, std::string> parents;
  for (const auto &t : s.transforms) {
    EXPECT_TRUE(ValidTransform(t));
    EXPECT_TRUE(parents.emplace(t.child, t.parent).second);
  }
  for (const auto &t : s.transforms) {
    std::set<std::string> visited;
    auto current = t.child;
    while (parents.count(current)) {
      ASSERT_TRUE(visited.insert(current).second);
      current = parents.at(current);
    }
    EXPECT_EQ(current, "aft_mapped");
  }
}
TEST(TransformGeometry, PreservesIndependentBranchesAndRejectsBadCalibration) {
  Fixture f;
  f.lidar.imu_extrinsic.reset();
  auto s = BuildSensorTransforms(&f.lidar, &f.rig);
  ASSERT_EQ(s.transforms.size(), 4u);
  EXPECT_FALSE(s.diagnostics.empty());
  for (const auto &t : s.transforms)
    EXPECT_EQ(t.parent, "orbvi_link");
  f = Fixture{};
  f.rig.cameras[1].rotation_cam_to_reference[0] =
      std::numeric_limits<double>::quiet_NaN();
  s = BuildSensorTransforms(&f.lidar, &f.rig);
  EXPECT_EQ(s.transforms.size(), 6u);
  EXPECT_FALSE(s.diagnostics.empty());
  f = Fixture{};
  f.lidar.camera_extrinsics.clear();
  s = BuildSensorTransforms(&f.lidar, &f.rig);
  EXPECT_EQ(s.transforms.size(), 6u);
  EXPECT_FALSE(s.diagnostics.empty());
  f = Fixture{};
  f.lidar.imu_extrinsic->translation_unit = "mm";
  s = BuildSensorTransforms(&f.lidar, &f.rig);
  EXPECT_EQ(s.transforms.size(), 4u);
  EXPECT_FALSE(s.diagnostics.empty());
  f = Fixture{};
  f.lidar.imu_extrinsic->rotation_matrix = {-1, 0, 0, 0, 1, 0, 0, 0, 1};
  s = BuildSensorTransforms(&f.lidar, &f.rig);
  EXPECT_EQ(s.transforms.size(), 4u);
}
TEST(TransformGeometry,
     DuplicateIdentityAndFrameCollisionDoNotCreateInvalidTree) {
  Fixture f;
  f.rig.cameras.push_back(f.rig.cameras.front());
  auto s = BuildSensorTransforms(&f.lidar, &f.rig);
  EXPECT_FALSE(s.diagnostics.empty());
  f = Fixture{};
  s = BuildSensorTransforms(&f.lidar, &f.rig, "aft_mapped");
  EXPECT_FALSE(s.diagnostics.empty());
  for (const auto &t : s.transforms)
    EXPECT_NE(t.parent, t.child);
  EXPECT_EQ(RawCameraFrameId("003"), "orbvi/camera/3");
  EXPECT_TRUE(RawCameraFrameId("../3").empty());
}
TEST(TransformGeometry, DeviceFrameRenamePreservesGeometryAndLegacyNames) {
  Fixture f;
  const auto current = BuildSensorTransforms(&f.lidar, &f.rig);
  const auto legacy = BuildSensorTransforms(&f.lidar, &f.rig, "base_link");
  ASSERT_TRUE(current.diagnostics.empty());
  ASSERT_TRUE(legacy.diagnostics.empty());
  ASSERT_EQ(current.transforms.size(), legacy.transforms.size());
  for (std::size_t i = 0; i < current.transforms.size(); ++i) {
    const auto &a = current.transforms[i];
    const auto &b = legacy.transforms[i];
    EXPECT_EQ(a.translation, b.translation);
    EXPECT_EQ(a.quaternion, b.quaternion);
    EXPECT_EQ(a.parent == "orbvi_link" ? "base_link" : a.parent, b.parent);
    EXPECT_EQ(a.child == "orbvi_link" ? "base_link" : a.child, b.child);
    EXPECT_NE(a.parent, "base_link");
    EXPECT_NE(a.child, "base_link");
  }
}
TEST(TransformGeometry, ExportContainsHashesAndMeterValues) {
  Fixture f;
  const auto text = ExtrinsicsJson(BuildSensorTransforms(&f.lidar, &f.rig));
  EXPECT_NE(text.find("\"translation_unit\":\"m\""), std::string::npos);
  EXPECT_NE(text.find("camera-hash"), std::string::npos);
  EXPECT_NE(text.find("lidar-hash"), std::string::npos);
  EXPECT_NE(text.find("orbvi/camera/3"), std::string::npos);
  EXPECT_EQ(BuildSensorTransforms(&f.lidar, &f.rig).transforms[0].quaternion[3],
            1);
}
TEST(PoseTransformGate,
     RejectsTimeAndSequenceDuplicatesAndRecoversForwardRestart) {
  orbvi_sdk::MetadataMap metadata{{"sequence", "10"}};
  orbvi_sdk::FrameView frame;
  frame.timestamp_ns = 100;
  frame.timestamp_domain = orbvi_sdk::TimestampDomain::Realtime;
  frame.metadata = &metadata;
  PoseTransformGate gate;
  EXPECT_TRUE(gate.Accept(frame));
  EXPECT_FALSE(gate.Accept(frame));
  frame.timestamp_ns = 99;
  metadata["sequence"] = "11";
  EXPECT_FALSE(gate.Accept(frame));
  frame.timestamp_ns = 101;
  EXPECT_TRUE(gate.Accept(frame));
  metadata["sequence"] = "1";
  frame.timestamp_ns = 102;
  bool changed = false;
  EXPECT_TRUE(gate.Accept(frame, &changed));
  EXPECT_TRUE(changed);
  frame.timestamp_domain = orbvi_sdk::TimestampDomain::Unknown;
  frame.timestamp_ns = 103;
  EXPECT_FALSE(gate.Accept(frame));
  frame.timestamp_domain = orbvi_sdk::TimestampDomain::Realtime;
  metadata["timestamp_valid"] = "false";
  EXPECT_FALSE(gate.Accept(frame));
}
TEST(PoseTransformGate, EpochChangeAllowsResetButRetiredEpochDoesNotReturn) {
  orbvi_sdk::MetadataMap metadata{
      {"sequence", "10"}, {"boot_id", "1"}, {"generation", "1"}};
  orbvi_sdk::FrameView frame;
  frame.timestamp_ns = 100;
  frame.timestamp_domain = orbvi_sdk::TimestampDomain::Realtime;
  frame.metadata = &metadata;
  PoseTransformGate gate;
  ASSERT_TRUE(gate.Accept(frame));
  metadata["generation"] = "2";
  metadata["sequence"] = "1";
  frame.timestamp_ns = 50;
  bool changed = false;
  ASSERT_TRUE(gate.Accept(frame, &changed));
  EXPECT_TRUE(changed);
  metadata["generation"] = "1";
  frame.timestamp_ns = 101;
  EXPECT_FALSE(gate.Accept(frame));
}

TEST(TransformGeometry, HalfTurnRotationAndInvalidQuaternion) {
  Fixture f;
  f.lidar.imu_extrinsic->rotation_matrix = {1, 0, 0, 0, -1, 0, 0, 0, -1};
  const auto s = BuildSensorTransforms(&f.lidar, &f.rig);
  ASSERT_TRUE(s.diagnostics.empty());
  EXPECT_NEAR(std::abs(Find(s, "livox_frame").quaternion[0]), 1, 1e-12);
  TransformRecord t{"a", "b", {0, 0, 0}, {0, 0, 0, 0}};
  EXPECT_FALSE(ValidTransform(t));
  t.quaternion[3] = 1;
  EXPECT_TRUE(ValidTransform(t));
  t.translation[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(ValidTransform(t));
}

TEST(TransformGeometry, RejectsMixedCalibrationHashesAndDynamicRootCollision) {
  Fixture f;
  f.lidar.camera_calibration_hash = "other-camera-hash";
  auto s = BuildSensorTransforms(&f.lidar, &f.rig);
  EXPECT_EQ(s.transforms.size(), 6u);
  EXPECT_NE(std::find(s.diagnostics.begin(), s.diagnostics.end(),
                      "camera_calibration_hash_mismatch"),
            s.diagnostics.end());
  f = Fixture{};
  s = BuildSensorTransforms(&f.lidar, &f.rig, "camera_init");
  EXPECT_EQ(s.transforms.size(), 2u);
}

TEST(TransformGeometry, IncompleteSdkBundleKeepsValidLidarImuBranch) {
  orbvi_sdk::CalibrationStatus calibration;
  calibration.calibration_hash = "camera-hash";
  calibration.raw_json = R"({
    "calibration_hash":"camera-hash",
    "lidar_calibration":{"status":"READY","lidar_model":"MID360",
      "lidar_calibration_hash":"lidar-hash","lidar_calibration_source":"test"},
    "lidar_extrinsics":[], "lidar_imu_extrinsics_available":true,
    "lidar_imu_extrinsic":{"schema_version":1,"sensor_model":"MID360",
      "parameter_source":"manufacturer_nominal","source_frame":"mid360_lidar",
      "target_frame":"mid360_imu","translation_unit":"m",
      "rotation_matrix":[0,-1,0,1,0,0,0,0,1],"translation_m":[0.1,0.2,0.3]}})";
  auto snapshot = LoadSensorTransforms(calibration);
  ASSERT_EQ(snapshot.transforms.size(), 2u);
  EXPECT_EQ(snapshot.lidar_hash, "lidar-hash");
  EXPECT_NEAR(Find(snapshot, "livox_frame").translation[2], .3, 1e-12);
  EXPECT_FALSE(snapshot.diagnostics.empty());
  calibration.calibration_hash = "different-snapshot";
  EXPECT_TRUE(LoadSensorTransforms(calibration).transforms.empty());
  calibration.calibration_hash = "camera-hash";
  auto position = calibration.raw_json.find("[0,-1,0,1,0,0,0,0,1]");
  calibration.raw_json.replace(position, 20, "[2,0,0,0,1,0,0,0,1]");
  EXPECT_TRUE(LoadSensorTransforms(calibration).transforms.empty());
}

TEST(PoseTransformGate, ReportsProducerDegradationWithoutConsumingRecoverySample) {
  orbvi_sdk::MetadataMap metadata{{"sequence", "10"},
      {"tracking_state", "TRACKING"}};
  orbvi_sdk::FrameView frame;
  frame.timestamp_ns = 100;
  frame.timestamp_domain = orbvi_sdk::TimestampDomain::Realtime;
  frame.metadata = &metadata;
  PoseTransformGate gate;
  std::string reason;
  ASSERT_TRUE(gate.Accept(frame, nullptr, &reason));
  EXPECT_TRUE(reason.empty());
  metadata["tracking_state"] = "DEGRADED";
  metadata["tracking_reason"] = "recovering";
  metadata["sequence"] = "11";
  frame.timestamp_ns = 101;
  bool changed = true;
  EXPECT_FALSE(gate.Accept(frame, &changed, &reason));
  EXPECT_FALSE(changed);
  EXPECT_EQ(reason, "tracking_state=DEGRADED; tracking_reason=recovering");
  metadata["tracking_state"] = "TRACKING";
  EXPECT_TRUE(gate.Accept(frame, nullptr, &reason));
  EXPECT_TRUE(reason.empty());
  EXPECT_FALSE(gate.Accept(frame, nullptr, &reason));
  EXPECT_EQ(reason, "duplicate_or_out_of_order_sample_timestamp");
  frame.timestamp_ns = 102;
  EXPECT_FALSE(gate.Accept(frame, nullptr, &reason));
  EXPECT_EQ(reason, "duplicate_or_out_of_order_sequence");
  metadata["sequence"] = "12";
  frame.timestamp_domain = orbvi_sdk::TimestampDomain::Unknown;
  EXPECT_FALSE(gate.Accept(frame, nullptr, &reason));
  EXPECT_EQ(reason, "non_realtime_sample_timestamp");
  frame.timestamp_domain = orbvi_sdk::TimestampDomain::Realtime;
  metadata["timestamp_valid"] = "false";
  EXPECT_FALSE(gate.Accept(frame, nullptr, &reason));
  EXPECT_EQ(reason, "producer_timestamp_invalid");
  metadata["timestamp_valid"] = "true";
  EXPECT_TRUE(gate.Accept(frame, nullptr, &reason));
  EXPECT_TRUE(reason.empty());
}
