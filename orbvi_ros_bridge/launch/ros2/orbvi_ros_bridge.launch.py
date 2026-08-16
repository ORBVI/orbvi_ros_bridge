from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENT_DEFAULTS = {
    "host": "127.0.0.1",
    "streams": "raw,rectified,imu,disparity,depth,vio",
    "publish_lidar_pcl": "false",
    "publish_ipm": "false",
    "depth_viz_show_grid_values": "false",
    "publish_vio_path": "true",
    "vio_path_max_poses": "2000",
    "vio_path_publish_rate_hz": "10.0",
}

DEFAULT_BRIDGE_PARAMETERS = {
    "streams": "raw,rectified,imu,disparity,depth,vio",
}


def bridge_launch_description(parameters, extra_parameter_files=None):
    fastdds_shm_profile = PathJoinSubstitution([
        FindPackageShare("orbvi_ros_bridge"),
        "config",
        "fastdds_shm.xml",
    ])
    node_parameters = list(extra_parameter_files or [])
    node_parameters.append(parameters)
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value=ARGUMENT_DEFAULTS["host"]),
        DeclareLaunchArgument("streams", default_value=ARGUMENT_DEFAULTS["streams"]),
        DeclareLaunchArgument(
            "publish_lidar_pcl",
            default_value=ARGUMENT_DEFAULTS["publish_lidar_pcl"]),
        DeclareLaunchArgument(
            "publish_ipm",
            default_value=ARGUMENT_DEFAULTS["publish_ipm"]),
        DeclareLaunchArgument(
            "depth_viz_show_grid_values",
            default_value=ARGUMENT_DEFAULTS["depth_viz_show_grid_values"]),
        DeclareLaunchArgument(
            "publish_vio_path",
            default_value=ARGUMENT_DEFAULTS["publish_vio_path"]),
        DeclareLaunchArgument(
            "vio_path_max_poses",
            default_value=ARGUMENT_DEFAULTS["vio_path_max_poses"]),
        DeclareLaunchArgument(
            "vio_path_publish_rate_hz",
            default_value=ARGUMENT_DEFAULTS["vio_path_publish_rate_hz"]),
        SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_fastrtps_cpp"),
        SetEnvironmentVariable(name="FASTRTPS_DEFAULT_PROFILES_FILE", value=fastdds_shm_profile),
        SetEnvironmentVariable(name="FASTDDS_DEFAULT_PROFILES_FILE", value=fastdds_shm_profile),
        SetEnvironmentVariable(name="RMW_FASTRTPS_PUBLICATION_MODE", value="ASYNCHRONOUS"),
        Node(
            package="orbvi_ros_bridge",
            executable="orbvi_ros_bridge_node",
            name="orbvi_ros_bridge",
            output="screen",
            parameters=node_parameters,
        ),
    ])


def generate_launch_description():
    parameters = dict(DEFAULT_BRIDGE_PARAMETERS)
    parameters["host"] = LaunchConfiguration("host")
    parameters["streams"] = LaunchConfiguration("streams")
    parameters["publish_lidar_pcl"] = LaunchConfiguration("publish_lidar_pcl")
    parameters["publish_ipm"] = LaunchConfiguration("publish_ipm")
    parameters["depth_viz_show_grid_values"] = LaunchConfiguration(
        "depth_viz_show_grid_values")
    parameters["publish_vio_path"] = LaunchConfiguration("publish_vio_path")
    parameters["vio_path_max_poses"] = LaunchConfiguration("vio_path_max_poses")
    parameters["vio_path_publish_rate_hz"] = LaunchConfiguration(
        "vio_path_publish_rate_hz")
    return bridge_launch_description(parameters)
