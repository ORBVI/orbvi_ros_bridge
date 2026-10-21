from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare("orbvi_ros_bridge"),
        "visuallize_config",
        "rviz-config-ros2.rviz",
    ])
    fastdds_shm_profile = PathJoinSubstitution([
        FindPackageShare("orbvi_ros_bridge"),
        "config",
        "fastdds_shm.xml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "fixed_frame",
            default_value="camera_init",
            description="Fixed frame: orbvi_link for depth, livox_frame for LiDAR; poses use their header frame (4VL: camera_init).",
        ),
        SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_fastrtps_cpp"),
        SetEnvironmentVariable(name="FASTRTPS_DEFAULT_PROFILES_FILE", value=fastdds_shm_profile),
        SetEnvironmentVariable(name="FASTDDS_DEFAULT_PROFILES_FILE", value=fastdds_shm_profile),
        SetEnvironmentVariable(name="RMW_FASTRTPS_PUBLICATION_MODE", value="ASYNCHRONOUS"),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config, '-f', LaunchConfiguration("fixed_frame")],
            output='screen',
        ),
    ])
