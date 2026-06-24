from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENT_DEFAULTS = {
    "host": "127.0.0.1",
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
    return bridge_launch_description(parameters)
