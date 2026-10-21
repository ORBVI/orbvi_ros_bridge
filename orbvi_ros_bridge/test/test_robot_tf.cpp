#include "robot_tf.hpp"
#include "robot_tf_session.hpp"
#include <cmath>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <set>
#include <unistd.h>

using namespace orbvi_ros_bridge;
namespace {
constexpr double pi = 3.14159265358979323846;
SensorTransforms Device() {
  SensorTransforms s;
  s.camera_hash = "rig";
  s.lidar_hash = "lidar";
  s.transforms = {{"aft_mapped", "orbvi_link", {1,2,3}, {1,0,0,0}},
                  {"aft_mapped", "mid360_imu", {}, {0,0,0,1}},
                  {"aft_mapped", "livox_frame", {.1,.2,.3}, {0,0,0,1}}};
  for (int id : {3,2,4,5})
    s.transforms.push_back({"orbvi_link",RawCameraFrameId(std::to_string(id)),
                            {double(id),0,0},{0,0,0,1}});
  return s;
}
const TransformRecord &Find(const SensorTransforms &s, const std::string &child) {
  for (const auto &t : s.transforms) if (t.child == child) return t;
  throw std::runtime_error("Missing " + child);
}
struct TempYaml {
  std::string path;
  explicit TempYaml(const std::string &text) {
    char file[] = "/tmp/orbvi-mount-test-XXXXXX";
    const int fd = mkstemp(file);
    if (fd < 0) throw std::runtime_error("mkstemp");
    close(fd);
    path = file;
    std::ofstream(path) << text;
  }
  ~TempYaml() { unlink(path.c_str()); }
};
} // namespace

TEST(RobotTf, RpyUsesFixedAxesAndSupportsInvertedAndSideMounts) {
  for (const auto &rpy : {std::array<double,3>{0,0,0}, {pi,0,0}, {0,pi/2,0}, {.3,-.4,.7}}) {
    const auto m = MakeRobotMounting("base_link","orbvi_link",{.2,.3,.4},rpy);
    const std::array<double,3> p{{2,3,4}};
    const double cr=cos(rpy[0]),sr=sin(rpy[0]),cp=cos(rpy[1]),sp=sin(rpy[1]),cy=cos(rpy[2]),sy=sin(rpy[2]);
    // Independently apply Rx, then Ry, then Rz to a known point.
    const double ry=cr*p[1]-sr*p[2], rz=sr*p[1]+cr*p[2];
    const double px=cp*p[0]+sp*rz, pz=-sp*p[0]+cp*rz;
    const auto result = TransformPoint(m.base_from_device,p);
    EXPECT_NEAR(result[0],cy*px-sy*ry+.2,1e-12);
    EXPECT_NEAR(result[1],sy*px+cy*ry+.3,1e-12);
    EXPECT_NEAR(result[2],pz+.4,1e-12);
  }
}
TEST(RobotTf, RobotPoseIncludesRotationAndTranslationLeverArm) {
  const auto device = Device();
  const auto mount = MakeRobotMounting("base_link","orbvi_link",{.2,.3,.4},{0,0,pi/2});
  const auto robot = BuildRobotTransforms(device,mount,false);
  ASSERT_TRUE(robot.imu_from_base);
  TransformRecord result;
  const double h = std::sqrt(.5);
  ASSERT_TRUE(RobotPoseTransform({"camera_init","aft_mapped",{10,20,30},{0,0,h,h}},
                                *robot.imu_from_base,&result));
  EXPECT_EQ(result.parent,"camera_init");
  EXPECT_EQ(result.child,"base_link");
  EXPECT_NEAR(result.translation[0],8.2,1e-12);
  EXPECT_NEAR(result.translation[1],20.7,1e-12);
  EXPECT_NEAR(result.translation[2],33.4,1e-12);
  const auto point = TransformPoint(result,{1,2,3});
  EXPECT_NEAR(point[0],7.2,1e-12);
  EXPECT_NEAR(point[1],22.7,1e-12);
  EXPECT_NEAR(point[2],30.4,1e-12);
}
TEST(RobotTf, ReparentedSensorsPreserveGeometryAndHaveOneOwner) {
  const auto input=Device();
  const auto mount=MakeRobotMounting("base_link","orbvi_link",{.2,0,.35},{pi,0,.2});
  auto robot=BuildRobotTransforms(input,mount,false);
  EXPECT_EQ(robot.sensors.camera_hash,input.camera_hash);
  EXPECT_EQ(robot.sensors.lidar_hash,input.lidar_hash);
  ASSERT_EQ(robot.sensors.transforms.size(),7u);
  EXPECT_EQ(Find(robot.sensors,"aft_mapped").parent,"mid360_imu");
  const auto lidar_in_device=TransformPoint(Find(robot.sensors,"livox_frame"),{4,5,6});
  const auto lidar_in_imu=TransformPoint(Find(input,"orbvi_link"),lidar_in_device);
  EXPECT_NEAR(lidar_in_imu[0],4.1,1e-12);
  EXPECT_NEAR(lidar_in_imu[1],5.2,1e-12);
  EXPECT_NEAR(lidar_in_imu[2],6.3,1e-12);
  for (const auto &t : robot.sensors.transforms) {
    EXPECT_NE(t.child,"orbvi_link");
    EXPECT_NE(t.parent,"aft_mapped");
  }
  robot=BuildRobotTransforms(input,mount,true);
  ASSERT_EQ(robot.sensors.transforms.size(),8u);
  EXPECT_EQ(Find(robot.sensors,"orbvi_link").parent,"base_link");
  std::map<std::string,std::string> parents;
  for (const auto &t : robot.sensors.transforms) {
    EXPECT_TRUE(ValidTransform(t));
    ASSERT_TRUE(parents.emplace(t.child,t.parent).second);
  }
  for (const auto &t : robot.sensors.transforms) {
    auto current=t.child;
    std::set<std::string> seen;
    while (parents.count(current)) {
      ASSERT_TRUE(seen.insert(current).second);
      current=parents.at(current);
    }
    EXPECT_EQ(current,"base_link");
  }
}
TEST(RobotTf, MissingAnchorDoesNotInventRobotPoseOrLoseIndependentBranches) {
  auto device=Device();
  device.transforms.erase(device.transforms.begin());
  const auto mount=MakeRobotMounting("base_link","orbvi_link",{0,0,1},{0,0,0});
  const auto robot=BuildRobotTransforms(device,mount,true);
  EXPECT_FALSE(robot.imu_from_base);
  EXPECT_FALSE(robot.sensors.diagnostics.empty());
  ASSERT_EQ(robot.sensors.transforms.size(),7u);
  EXPECT_EQ(Find(robot.sensors,"mid360_imu").parent,"aft_mapped");
  EXPECT_EQ(Find(robot.sensors,"livox_frame").parent,"aft_mapped");
  for (const auto &t : robot.sensors.transforms) {
    EXPECT_NE(t.child,"aft_mapped");
  }
}
TEST(RobotTf, InvalidFramesAndNonfiniteDataAreRejected) {
  for (const auto &frame : {"camera_init","aft_mapped","livox_frame","mid360_imu","orbvi/camera/3","/base_link",""})
    EXPECT_THROW(MakeRobotMounting(frame,"orbvi_link",{0,0,1},{0,0,0}),std::exception);
  EXPECT_THROW(MakeRobotMounting("base_link","base_link",{0,0,1},{0,0,0}),std::exception);
  EXPECT_THROW(MakeRobotMounting("base_link","orbvi_link",{0,0,1},
                               {0,0,std::numeric_limits<double>::infinity()}),std::exception);
  EXPECT_THROW(MakeRobotMounting("base_link","orbvi_link",{NAN,0,1},{0,0,0}),std::exception);
  TransformRecord output;
  const auto correction=BuildRobotTransforms(Device(),MakeRobotMounting("base_link","orbvi_link",{0,0,1},{0,0,0}),false).imu_from_base;
  ASSERT_TRUE(correction);
  EXPECT_FALSE(RobotPoseTransform({"world","body",{},{0,0,0,1}},*correction,&output));
  EXPECT_FALSE(RobotPoseTransform({"camera_init","aft_mapped",{},{0,0,0,0}},*correction,&output));
  auto huge = Device();
  huge.transforms[0].translation = {1e308,0,0};
  huge.transforms[0].quaternion = {0,0,0,1};
  const auto overflow = BuildRobotTransforms(huge,
      MakeRobotMounting("base_link","orbvi_link",{-1e308,0,0},{0,0,0}),false);
  EXPECT_FALSE(overflow.imu_from_base);
  EXPECT_FALSE(overflow.sensors.diagnostics.empty());
  for (const auto &t : overflow.sensors.transforms) EXPECT_TRUE(ValidTransform(t));
}
TEST(RobotTf, MountingYamlAndModeValidationAreStrict) {
  const std::string valid="orbvi:\n  mounting:\n    parent_frame: base_link\n    child_frame: orbvi_link\n    xyz: [0.2, 0, 0.35]\n    rpy: [3.141592653589793, 0, 0]\n  ground_plane: [0, 0, 1, 0]\n";
  TempYaml yaml(valid);
  const auto setup=ConfigureTf("robot",yaml.path,true,"");
  ASSERT_TRUE(setup.mounting);
  ASSERT_TRUE(setup.mounting->ground_plane);
  EXPECT_EQ(setup.device_frame,"orbvi_link");
  EXPECT_NEAR(setup.mounting->base_from_device.quaternion[0],1,1e-12);
  EXPECT_THROW(ConfigureTf("robot","",false,""),std::exception);
  EXPECT_THROW(ConfigureTf("robot",yaml.path,false,"base_link"),std::exception);
  EXPECT_THROW(ConfigureTf("device",yaml.path,false,""),std::exception);
  EXPECT_THROW(ConfigureTf("device","",true,""),std::exception);
  EXPECT_THROW(ConfigureTf("unknown","",false,""),std::exception);
  EXPECT_EQ(ConfigureTf("device","",false,"base_link").device_frame,"base_link");
  for (auto text : {std::string("orbvi: {}"), valid+"orbvi: {}\n", std::string(65537,' '),
                    std::string("orbvi: {mounting: {parent_frame: base_link, child_frame: orbvi_link, xyz: [0,0], rpy: [0,0,0]}}")}) {
    TempYaml bad(text);
    EXPECT_THROW(LoadRobotMounting(bad.path),std::exception);
  }
}

TEST(RobotTf, UrdfModeRejectsASecondInstallationPose) {
  TempYaml duplicate("orbvi: {mounting: {parent_frame: base_link, child_frame: orbvi_link, xyz: [1,2,3], rpy: [0,0,0]}}");
  EXPECT_THROW(ConfigureTf("robot",duplicate.path,false,""),std::exception);
  TempYaml urdf("orbvi: {mounting: {parent_frame: base_link, child_frame: orbvi_link}, ground_plane: [0,0,1,0]}");
  auto config=ConfigureTf("robot",urdf.path,false,"");
  ASSERT_TRUE(config.mounting);EXPECT_FALSE(config.mounting->pose_from_config);
  EXPECT_FALSE(BuildRobotTransforms(Device(),*config.mounting,false).imu_from_base);
  EXPECT_THROW(BuildRobotTransforms(Device(),*config.mounting,true),std::exception);
  EXPECT_THROW(ConfigureTf("robot",urdf.path,true,""),std::exception);
}
TEST(RobotTf, StaticPathsRejectDuplicateOwnersChangesDynamicsAndCycles) {
  const TransformRecord a{"base","bracket",{1,0,0},{0,0,0,1}},b{"bracket","sensor",{0,2,0},{0,0,0,1}};
  StaticTfRegistry registry;TransformRecord result;std::string error;
  registry.Observe(a,"@local");registry.Observe(a,"urdf");registry.Observe(b,"sdk");
  ASSERT_TRUE(registry.Resolve("base","sensor",&result,&error));
  EXPECT_DOUBLE_EQ(result.translation[0],1);EXPECT_DOUBLE_EQ(result.translation[1],2);
  auto repeated=a;for(auto &q:repeated.quaternion)q=-q;
  registry.Observe(repeated,"urdf");EXPECT_TRUE(registry.Resolve("sensor","base",&result,&error));
  auto duplicate=registry;duplicate.Observe(a,"another_node");EXPECT_FALSE(duplicate.Resolve("base","sensor",&result,&error));
  auto changed=registry;repeated.translation[0]=2;changed.Observe(repeated,"urdf");EXPECT_FALSE(changed.Resolve("base","sensor",&result,&error));
  auto moving=registry;moving.ObserveDynamic("bracket");EXPECT_FALSE(moving.Resolve("base","sensor",&result,&error));
  registry.Observe({"sensor","base",{}, {0,0,0,1}},"bad");EXPECT_FALSE(registry.Resolve("base","sensor",&result,&error));
}
TEST(RobotTf, StartupOdomUsesGroundProjectionAndRobotYaw) {
  const double h=std::sqrt(.5);
  const TransformRecord pose{"camera_init","base_link",{3,4,2},{0,0,h,h}};
  const auto odom=StartupOdomTransform(pose,{0,0,1,.5},"odom");ASSERT_TRUE(odom);
  const auto first=ComposeTransforms(*odom,pose);
  EXPECT_NEAR(first.translation[0],0,1e-12);EXPECT_NEAR(first.translation[1],0,1e-12);
  EXPECT_NEAR(first.translation[2],.5,1e-12);EXPECT_NEAR(first.quaternion[3],1,1e-12);
  const auto ground=TransformPoint(first,{0,0,-.5});EXPECT_NEAR(ground[2],0,1e-12);
  EXPECT_FALSE(StartupOdomTransform(pose,{1,0,0,0},"odom"));
  EXPECT_FALSE(StartupOdomTransform(pose,{0,0,1,0},"base_link"));
  EXPECT_FALSE(StartupOdomTransform(pose,{0,0,1,0},"livox_frame"));
}
TEST(RobotTf, StartupOdomKeepsWorldVerticalAndActualRobotTilt) {
  auto pose=MakeRobotMounting("world","robot",{3,4,2},{0,pi/6,pi/2}).base_from_device;
  pose.parent="camera_init";pose.child="base_link";
  // A world-horizontal plane 0.5 m beneath the tilted robot origin.
  const std::array<double,4> plane{-0.5,0,std::sqrt(.75),.5};
  const auto odom=StartupOdomTransform(pose,plane,"odom");ASSERT_TRUE(odom);
  EXPECT_DOUBLE_EQ(odom->quaternion[0],0);EXPECT_DOUBLE_EQ(odom->quaternion[1],0);
  const auto first=ComposeTransforms(*odom,pose);
  EXPECT_NEAR(first.translation[0],0,1e-12);EXPECT_NEAR(first.translation[1],0,1e-12);
  EXPECT_NEAR(first.translation[2],.5,1e-12);
  const auto front=TransformPoint(first,{1,0,0});
  EXPECT_NEAR(front[0],std::sqrt(.75),1e-12);EXPECT_NEAR(front[1],0,1e-12);
  EXPECT_NEAR(front[2],0,1e-12); // Pitch remains 30 degrees.
  for (const auto &point:{std::array<double,3>{1,0,0},{0,0,-.5/std::sqrt(.75)}})
    EXPECT_NEAR(TransformPoint(first,point)[2],0,1e-12);
}
namespace {
struct SessionFixture {
  StaticTfRegistry buffer;
  std::vector<TransformRecord> broadcasts;
  TfSetup setup;
  SessionFixture(bool bridge) {
    auto mounting=MakeRobotMounting("base_link","orbvi_link",{.2,0,.35},{0,0,.6});
    mounting.ground_plane=std::array<double,4>{0,0,1,.5};
    mounting.pose_from_config=bridge;
    setup.device_frame="orbvi_link";setup.mounting=mounting;setup.publish_mounting_tf=bridge;
  }
  std::unique_ptr<RobotTfSession> Create(bool sensors=true,bool odom=false) {
    return std::make_unique<RobotTfSession>(setup,BuildRobotTransforms(Device(),*setup.mounting,false).sensors,sensors,odom,"odom",
      [this](const auto &p,const auto &c){TransformRecord t;std::string error;if(!buffer.Resolve(p,c,&t,&error))throw std::runtime_error(error);return t;},
      [this](const auto &t,bool broadcast){buffer.Observe(t,"@local");if(broadcast)broadcasts.push_back(t);},std::chrono::steady_clock::time_point{});
  }
  void Receive(RobotTfSession &session,const TransformRecord &t,const std::string &owner) {
    buffer.Observe(t,owner);session.Observe(t,owner,true);
  }
};
}
TEST(RobotTf, SessionWaitsForUrdfAndBothConsumersUseItsActualPose) {
  SessionFixture f(false);auto session=f.Create();session->Poll();EXPECT_FALSE(session->Snapshot());
  // This is intentionally unrelated to the frame-only URDF config's placeholder.
  const auto mount=MakeRobotMounting("base_link","orbvi_link",{4,5,6},{0,0,pi/2}).base_from_device;
  f.Receive(*session,mount,"robot_state_publisher");session->Poll();
  auto snapshot=session->Snapshot();ASSERT_TRUE(snapshot);ASSERT_TRUE(snapshot->imu_from_base);
  const auto &camera=Find(snapshot->cameras,RawCameraFrameId("3"));
  const auto point=TransformPoint(camera,{0,0,0});EXPECT_NEAR(point[0],4,1e-12);EXPECT_NEAR(point[1],8,1e-12);EXPECT_NEAR(point[2],6,1e-12);
  TransformRecord pose;ASSERT_TRUE(session->Pose({"camera_init","aft_mapped",{}, {0,0,0,1}},false,&pose));
  const auto roundtrip=ComposeTransforms(pose,mount);const auto expected=Find(Device(),"orbvi_link");
  for(int i=0;i<3;++i)EXPECT_NEAR(roundtrip.translation[i],expected.translation[i],1e-12);
  EXPECT_TRUE(f.broadcasts.empty());
  f.Receive(*session,mount,"duplicate_publisher");EXPECT_FALSE(session->Snapshot());
  EXPECT_FALSE(session->Pose({"camera_init","aft_mapped",{}, {0,0,0,1}},false,&pose));
}
TEST(RobotTf, BridgeRefusesExistingOwnerAndTfOffKeepsLocalGeometry) {
  SessionFixture f(true);auto session=f.Create();
  f.Receive(*session,f.setup.mounting->base_from_device,"urdf");session->Poll();
  EXPECT_FALSE(session->Snapshot());EXPECT_TRUE(f.broadcasts.empty());
  SessionFixture local(true);auto offline=local.Create(false);offline->Poll();
  ASSERT_TRUE(offline->Snapshot());EXPECT_TRUE(offline->Snapshot()->imu_from_base);EXPECT_TRUE(local.broadcasts.empty());
}
TEST(RobotTf, StartupAnchorIsPublishedOnceAndAlgorithmResetRequiresRestart) {
  SessionFixture f(true);auto session=f.Create(true,true);session->Poll();ASSERT_TRUE(session->Snapshot());
  ASSERT_EQ(f.broadcasts.size(),1u); // Installation only.
  TransformRecord out;
  ASSERT_TRUE(session->Pose({"camera_init","aft_mapped",{1,2,3},{0,0,0,1}},false,&out));
  ASSERT_EQ(f.broadcasts.size(),2u);EXPECT_EQ(f.broadcasts.back().parent,"odom");
  ASSERT_TRUE(session->Pose({"camera_init","aft_mapped",{2,2,3},{0,0,0,1}},false,&out));EXPECT_EQ(f.broadcasts.size(),2u);
  EXPECT_FALSE(session->Pose({"camera_init","aft_mapped",{}, {0,0,0,1}},true,&out));EXPECT_FALSE(session->Snapshot());
  EXPECT_EQ(f.broadcasts.size(),2u);
}
TEST(RobotTf, IndependentCameraSnapshotAdoptsALaterImuConnection) {
  SessionFixture f(true);
  auto device=Device();device.transforms.erase(device.transforms.begin());
  const auto sensors=BuildRobotTransforms(device,*f.setup.mounting,false).sensors;
  RobotTfSession session(f.setup,sensors,false,false,"odom",
      [&f](const auto &p,const auto &c){TransformRecord t;std::string error;if(!f.buffer.Resolve(p,c,&t,&error))throw std::runtime_error(error);return t;},
      [&f](const auto &t,bool){f.buffer.Observe(t,"@local");});
  session.Poll();ASSERT_TRUE(session.Snapshot());EXPECT_FALSE(session.Snapshot()->imu_from_base);
  f.Receive(session,InverseTransform(Find(Device(),"orbvi_link")),"calibration_source");
  session.Poll();ASSERT_TRUE(session.Snapshot());EXPECT_TRUE(session.Snapshot()->imu_from_base);
}
TEST(RobotTf, StartupOdomRejectsAnExistingFrameInsideTheRobotSubtree) {
  SessionFixture f(true);auto session=f.Create(true,true);session->Poll();
  f.Receive(*session,{"orbvi_link","odom",{}, {0,0,0,1}},"bad_urdf");
  TransformRecord out;
  EXPECT_FALSE(session->Pose({"camera_init","aft_mapped",{}, {0,0,0,1}},false,&out));
  EXPECT_EQ(f.broadcasts.size(),1u);EXPECT_FALSE(session->Snapshot());
}
