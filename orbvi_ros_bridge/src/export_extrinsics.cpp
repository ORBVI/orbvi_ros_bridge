#include "orbvi_sdk/client.hpp"
#include "board_ipm.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <unistd.h>

int main(int argc, char **argv) {
  orbvi_sdk::ClientOptions options;
  std::string output;
  std::string device_frame;
  std::string tf_mode = "device";
  std::string mounting_config;
  bool publish_mounting_tf = false;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (key == "--help") {
        std::cout << "orbvi_export_extrinsics --host HOST --control-port PORT "
                     "--output NEW_FILE [--device-frame FRAME] "
                     "[--tf-mode device|robot --mounting-config FILE|sdk "
                     "--publish-mounting-tf true|false]\n"
                     "Robot URDF mode: frame-only config; exports internal calibration only.\n"
                     "Bridge mode: true requires xyz/rpy config and includes mounting.\n"
                     "No ROS TF lookup; dynamic/startup odom edges are not exported.\n";
        return 0;
      }
      if (i + 1 == argc)
        throw std::runtime_error("Missing value for " + key);
      const std::string value = argv[++i];
      if (key == "--host")
        options.host = value;
      else if (key == "--output")
        output = value;
      else if (key == "--device-frame")
        device_frame = value;
      else if (key == "--tf-mode")
        tf_mode = value;
      else if (key == "--mounting-config")
        mounting_config = value;
      else if (key == "--publish-mounting-tf") {
        if (value != "true" && value != "false")
          throw std::runtime_error("--publish-mounting-tf must be true or false");
        publish_mounting_tf = value == "true";
      } else if (key == "--control-port") {
        std::size_t used = 0;
        const auto port = std::stoul(value, &used);
        if (used != value.size() || port == 0 || port > 65535)
          throw std::runtime_error("Invalid port");
        options.control_port = static_cast<std::uint16_t>(port);
      } else
        throw std::runtime_error("Unknown option: " + key);
    }
    if (output.empty())
      throw std::runtime_error(
          "--output is required (existing files are never overwritten)");
    orbvi_sdk::Client client(options);
    const auto connected = client.connect();
    if (!connected)
      throw std::runtime_error(connected.error().message);
    const auto setup = tf_mode=="robot" && mounting_config=="sdk"
        ? orbvi_ros_bridge::ConfigureBoardTf(orbvi_ros_bridge::ReadBoardMountingSnapshot(client,{}),publish_mounting_tf,device_frame)
        : orbvi_ros_bridge::ConfigureTf(tf_mode,mounting_config,publish_mounting_tf,device_frame);
    const auto calibration = client.getCalibration("");
    if (!calibration)
      throw std::runtime_error(calibration.error().message);
    auto snapshot =
        orbvi_ros_bridge::LoadSensorTransforms(calibration.value(), setup.device_frame);
    if (setup.mounting)
      snapshot = orbvi_ros_bridge::BuildRobotTransforms(
          snapshot, *setup.mounting, publish_mounting_tf).sensors;
    client.disconnect();
    const auto json = orbvi_ros_bridge::ExtrinsicsJson(snapshot);
    const int fd =
        ::open(output.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
    if (fd < 0)
      throw std::runtime_error(std::string("Cannot create output: ") +
                               std::strerror(errno));
    std::size_t written = 0;
    while (written < json.size()) {
      const auto count =
          ::write(fd, json.data() + written, json.size() - written);
      if (count < 0 && errno == EINTR)
        continue;
      if (count <= 0) {
        ::close(fd);
        ::unlink(output.c_str());
        throw std::runtime_error("Failed to write output");
      }
      written += static_cast<std::size_t>(count);
    }
    if (::close(fd) != 0) {
      ::unlink(output.c_str());
      throw std::runtime_error("Failed to close output");
    }
    for (const auto &message : snapshot.diagnostics)
      std::cerr << message << '\n';
    std::cout << "Exported " << snapshot.transforms.size() << " transforms to "
              << output << '\n';
    return snapshot.diagnostics.empty()
               ? 0
               : 3; // Partial snapshots remain reviewable.
  } catch (const std::exception &e) {
    std::cerr << "orbvi_export_extrinsics: " << e.what() << '\n';
    return 1;
  }
}
