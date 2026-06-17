from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENT_DEFAULTS = {
    "host": "127.0.0.1",
    "control_port": "18088",
    "connect_timeout_ms": "2000",
    "connect_retry_count": "4",
    "connect_retry_delay_ms": "1000",
    "streams": "raw,rectified,imu,lidar,lidar_imu,disparity,depth,vio",
    "topic_prefix": "/orbvi",
    "image_mode": "raw-and-decoded",
    "raw_camera_ids": "",
    "max_receive_queue_depth": "64",
    "max_decode_queue_depth": "64",
    "max_publish_queue_depth": "64",
    "queue_size": "10",
    "publish_worker_threads": "8",
    "sensor_data_best_effort": "true",
    "log_delivery_stats": "false",
    "use_fastdds_shm": "true",
}


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in ARGUMENT_DEFAULTS.items()
    ]
    arguments.append(DeclareLaunchArgument(
        "fastdds_shm_profile",
        default_value=PathJoinSubstitution([
            FindPackageShare("orbvi_ros_bridge"),
            "config",
            "fastdds_shm.xml",
        ]),
    ))
    launch_only_arguments = {"use_fastdds_shm", "fastdds_shm_profile"}
    parameters = {
        name: LaunchConfiguration(name)
        for name in ARGUMENT_DEFAULTS
        if name not in launch_only_arguments
    }
    use_fastdds_shm = IfCondition(LaunchConfiguration("use_fastdds_shm"))
    shm_environment = [
        SetEnvironmentVariable(
            name="RMW_IMPLEMENTATION",
            value="rmw_fastrtps_cpp",
            condition=use_fastdds_shm,
        ),
        SetEnvironmentVariable(
            name="FASTRTPS_DEFAULT_PROFILES_FILE",
            value=LaunchConfiguration("fastdds_shm_profile"),
            condition=use_fastdds_shm,
        ),
        SetEnvironmentVariable(
            name="FASTDDS_DEFAULT_PROFILES_FILE",
            value=LaunchConfiguration("fastdds_shm_profile"),
            condition=use_fastdds_shm,
        ),
        SetEnvironmentVariable(
            name="RMW_FASTRTPS_PUBLICATION_MODE",
            value="ASYNCHRONOUS",
            condition=use_fastdds_shm,
        ),
    ]
    return LaunchDescription(arguments + shm_environment + [
        Node(
            package="orbvi_ros_bridge",
            executable="orbvi_ros_bridge_node",
            name="orbvi_ros_bridge",
            output="screen",
            parameters=[parameters],
        )
    ])
