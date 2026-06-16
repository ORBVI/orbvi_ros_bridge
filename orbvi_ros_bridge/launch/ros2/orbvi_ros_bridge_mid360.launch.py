from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENT_DEFAULTS = {
    "host": "127.0.0.1",
    "control_port": "18088",
    "topic_prefix": "/orbvi",
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
    parameters.update({
        "streams": "lidar,lidar_imu",
        "image_mode": "raw-only",
        "publish_depth": "false",
    })
    return LaunchDescription(arguments + [
        Node(
            package="orbvi_ros_bridge",
            executable="orbvi_ros_bridge_node",
            name="orbvi_ros_bridge",
            output="screen",
            parameters=[parameters],
        )
    ])
