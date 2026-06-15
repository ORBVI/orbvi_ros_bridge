from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENT_DEFAULTS = {
    "host": "127.0.0.1",
    "control_port": "18088",
    "topic_prefix": "/orbvi",
    "require_streaming_transport": "true",
    "allow_sample_endpoint_fallback": "false",
    "queue_size": "4",
    "max_receive_queue_depth": "8",
    "first_frame_timeout_ms": "5000",
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
        "max_decode_queue_depth": "1",
        "publish_depth": "false",
        "publish_depth_viz": "false",
        "publish_depth_pointcloud": "false",
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
