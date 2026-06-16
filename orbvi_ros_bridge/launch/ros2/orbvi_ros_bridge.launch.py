from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENT_DEFAULTS = {
    "host": "127.0.0.1",
    "control_port": "18088",
    "streams": "all",
    "topic_prefix": "/orbvi",
    "image_mode": "raw-and-decoded",
    "queue_size": "4",
    "max_receive_queue_depth": "8",
    "max_decode_queue_depth": "8",
    "publish_depth": "auto",
    "publish_depth_viz": "true",
    "publish_depth_pointcloud": "true",
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
    return LaunchDescription(arguments + [
        Node(
            package="orbvi_ros_bridge",
            executable="orbvi_ros_bridge_node",
            name="orbvi_ros_bridge",
            output="screen",
            parameters=[parameters],
        )
    ])
