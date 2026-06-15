#pragma once

#include <memory>

#include <ros/ros.h>

namespace orbvi_ros_bridge {

class BridgeNode {
 public:
  BridgeNode(ros::NodeHandle node, ros::NodeHandle private_node);
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
