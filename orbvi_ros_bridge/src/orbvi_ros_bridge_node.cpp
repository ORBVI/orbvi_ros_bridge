#include <ros/ros.h>

#include "bridge_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "orbvi_ros_bridge");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  orbvi_ros_bridge::BridgeNode bridge(node, private_node);
  if (!bridge.Start()) {
    ROS_ERROR("orbvi_ros_bridge failed to start");
    return 1;
  }

  ros::spin();
  return 0;
}
