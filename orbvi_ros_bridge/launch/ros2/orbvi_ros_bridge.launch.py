from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENT_DEFAULTS = {
    "host": "127.0.0.1",
    "control_port": "18088",
    "connect_timeout_ms": "2000",
    "connect_retry_count": "4",
    "connect_retry_delay_ms": "1000",
}


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in ARGUMENT_DEFAULTS.items()
    ]
    parameters = {
        name: LaunchConfiguration(name)
        for name in ARGUMENT_DEFAULTS
    }
    parameters["streams"] = "raw,rectified,imu,lidar,lidar_imu,disparity,depth,vio"
    parameters["topic_prefix"] = "/orbvi"
    return LaunchDescription(arguments + [
        Node(
            package="orbvi_ros_bridge",
            executable="orbvi_ros_bridge_node",
            name="orbvi_ros_bridge",
            output="screen",
            parameters=[parameters],
        )
    ])
