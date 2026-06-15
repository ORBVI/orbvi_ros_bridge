#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace orbvi_ros_bridge {

class BridgeNode {
 public:
  explicit BridgeNode(rclcpp::Node::SharedPtr node);
  ~BridgeNode();

  bool Start();
  void Stop();

  BridgeNode(const BridgeNode&) = delete;
  BridgeNode& operator=(const BridgeNode&) = delete;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace orbvi_ros_bridge
