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
    "image_mode": "raw-and-decoded",
    "queue_size": "4",
    "max_receive_queue_depth": "8",
    "max_decode_queue_depth": "8",
    "max_decode_latency_ms": "100",
    "first_frame_timeout_ms": "5000",
    "publish_depth": "auto",
    "depth_format": "float32",
    "depth_invalid_policy": "nan",
    "depth_min_disparity_px": "0.01",
    "depth_max_depth_m": "50.0",
    "publish_depth_viz": "true",
    "publish_depth_pointcloud": "true",
    "depth_viz_min_depth_m": "0.1",
    "depth_viz_max_depth_m": "20.0",
    "depth_pointcloud_stride": "4",
    "depth_pointcloud_max_disp_jump_px": "2.0",
    "depth_pointcloud_frame": "ros",
    "depth_pointcloud_frame_id": "base_link",
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
    parameters["streams"] = "raw,rectified,imu,disparity,depth,vio"
    return LaunchDescription(arguments + [
        Node(
            package="orbvi_ros_bridge",
            executable="orbvi_ros_bridge_node",
            name="orbvi_ros_bridge",
            output="screen",
            parameters=[parameters],
        )
    ])
