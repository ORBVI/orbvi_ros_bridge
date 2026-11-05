from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare("orbvi_ros_bridge"),
        "visuallize_config",
        PythonExpression(["{'visual': 'rviz-visual-ros2.rviz', 'mid360': 'rviz-config-ros2.rviz'}['",
                          LaunchConfiguration("profile"), "']"]),
    ])
    fastdds_shm_profile = PathJoinSubstitution([
        FindPackageShare("orbvi_ros_bridge"),
        "config",
        "fastdds_shm.xml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="mid360", choices=["visual", "mid360"],
                              description="visual: depth cloud; mid360: accumulated registered RGB cloud"),
        DeclareLaunchArgument(
            "fixed_frame",
            default_value=PythonExpression([
                "{'visual': 'orbvi_link', 'mid360': 'camera_init'}['",
                LaunchConfiguration("profile"), "']"]),
            description="Fixed frame: orbvi_link for depth, livox_frame for LiDAR; poses use their header frame (4VL: camera_init).",
        ),
        SetEnvironmentVariable(
            name="RMW_IMPLEMENTATION",
            value=EnvironmentVariable("RMW_IMPLEMENTATION", default_value="rmw_fastrtps_cpp")),
        SetEnvironmentVariable(
            name="FASTRTPS_DEFAULT_PROFILES_FILE",
            value=EnvironmentVariable(
                "FASTRTPS_DEFAULT_PROFILES_FILE",
                default_value=EnvironmentVariable(
                    "FASTDDS_DEFAULT_PROFILES_FILE", default_value=fastdds_shm_profile))),
        SetEnvironmentVariable(
            name="FASTDDS_DEFAULT_PROFILES_FILE",
            value=EnvironmentVariable(
                "FASTDDS_DEFAULT_PROFILES_FILE",
                default_value=EnvironmentVariable(
                    "FASTRTPS_DEFAULT_PROFILES_FILE", default_value=fastdds_shm_profile))),
        SetEnvironmentVariable(
            name="RMW_FASTRTPS_PUBLICATION_MODE",
            value=EnvironmentVariable("RMW_FASTRTPS_PUBLICATION_MODE", default_value="ASYNCHRONOUS")),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config, '-f', LaunchConfiguration("fixed_frame")],
            output='screen',
        ),
    ])
