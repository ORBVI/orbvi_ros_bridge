from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENT_DEFAULTS = {
    "host": "127.0.0.1",
    "publish_lidar_pcl": "false",
}

DEFAULT_BRIDGE_PARAMETERS = {
    "streams": "raw,rectified,pano,imu,disparity,depth,vio",
}


def generate_launch_description():
    pano_config = PathJoinSubstitution([
        FindPackageShare("orbvi_ros_bridge"),
        "config",
        "pano_mode2_ros2.yaml",
    ])
    fastdds_shm_profile = PathJoinSubstitution([
        FindPackageShare("orbvi_ros_bridge"),
        "config",
        "fastdds_shm.xml",
    ])
    parameters = dict(DEFAULT_BRIDGE_PARAMETERS)
    parameters["host"] = LaunchConfiguration("host")
    parameters["publish_lidar_pcl"] = LaunchConfiguration("publish_lidar_pcl")
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value=ARGUMENT_DEFAULTS["host"]),
        DeclareLaunchArgument(
            "publish_lidar_pcl",
            default_value=ARGUMENT_DEFAULTS["publish_lidar_pcl"]),
        SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_fastrtps_cpp"),
        SetEnvironmentVariable(name="FASTRTPS_DEFAULT_PROFILES_FILE", value=fastdds_shm_profile),
        SetEnvironmentVariable(name="FASTDDS_DEFAULT_PROFILES_FILE", value=fastdds_shm_profile),
        SetEnvironmentVariable(name="RMW_FASTRTPS_PUBLICATION_MODE", value="ASYNCHRONOUS"),
        Node(
            package="orbvi_ros_bridge",
            executable="orbvi_ros_bridge_node",
            name="orbvi_ros_bridge",
            output="screen",
            parameters=[pano_config, parameters],
        ),
    ])
