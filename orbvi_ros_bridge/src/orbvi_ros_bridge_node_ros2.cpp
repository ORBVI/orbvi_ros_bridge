#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "bridge_node_ros2.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("orbvi_ros_bridge");

  orbvi_ros_bridge::BridgeNode bridge(node);
  if (!bridge.Start()) {
    RCLCPP_ERROR(node->get_logger(), "orbvi_ros_bridge failed to start");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
