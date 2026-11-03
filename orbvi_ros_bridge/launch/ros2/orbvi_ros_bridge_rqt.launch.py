"""rqt main window with the bridge's Fast DDS shared-memory transport settings.

Starting rqt from this launch avoids exporting the profile by hand. Add
Plugins > Topics > Topic Monitor to check topic types, rates and bandwidth, and
Plugins > Visualization > Image View to look at images.
"""
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fastdds_shm_profile = PathJoinSubstitution([
        FindPackageShare("orbvi_ros_bridge"),
        "config",
        "fastdds_shm.xml",
    ])

    return LaunchDescription([
        SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_fastrtps_cpp"),
        SetEnvironmentVariable(name="FASTRTPS_DEFAULT_PROFILES_FILE", value=fastdds_shm_profile),
        SetEnvironmentVariable(name="FASTDDS_DEFAULT_PROFILES_FILE", value=fastdds_shm_profile),
        SetEnvironmentVariable(name="RMW_FASTRTPS_PUBLICATION_MODE", value="ASYNCHRONOUS"),
        Node(
            package="rqt_gui",
            executable="rqt_gui",
            name="rqt",
            output="screen",
        ),
    ])
