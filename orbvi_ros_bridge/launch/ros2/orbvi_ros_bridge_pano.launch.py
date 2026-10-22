from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


ARGUMENT_DEFAULTS = {
    "host": "127.0.0.1",
    "streams": "raw,rectified,pano,bev,imu,disparity,depth,vio",
    "publish_lidar_pcl": "false",
    "sensor_data_best_effort": "false",
    "ipm_extent_m": "10.0",
    "ipm_resolution_m_per_pixel": "0.02",
    "ipm_max_publish_rate_hz": "5.0",
    "ipm_frame_id": "",
    "publish_ipm": "false",
    "depth_viz_show_grid_values": "false",
    "publish_vio_path": "true",
    "publish_vio_tf": "true",
    "depth_pointcloud_frame_id": "",
    "tf_mode": "device",
    "mounting_config": "",
    "publish_mounting_tf": "false",
    "publish_odom_tf": "false",
    "odom_frame": "odom",
    "vio_path_max_poses": "2000",
    "vio_path_publish_rate_hz": "10.0",
}

DEFAULT_BRIDGE_PARAMETERS = {
    "streams": "raw,rectified,pano,bev,imu,disparity,depth,vio",
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
    parameters["streams"] = LaunchConfiguration("streams")
    parameters["publish_lidar_pcl"] = LaunchConfiguration("publish_lidar_pcl")
    parameters["sensor_data_best_effort"] = LaunchConfiguration("sensor_data_best_effort")
    parameters["ipm_extent_m"] = LaunchConfiguration("ipm_extent_m")
    parameters["ipm_resolution_m_per_pixel"] = LaunchConfiguration("ipm_resolution_m_per_pixel")
    parameters["ipm_max_publish_rate_hz"] = LaunchConfiguration("ipm_max_publish_rate_hz")
    parameters["ipm_frame_id"] = ParameterValue(LaunchConfiguration("ipm_frame_id"), value_type=str)
    parameters["publish_ipm"] = LaunchConfiguration("publish_ipm")
    parameters["depth_viz_show_grid_values"] = LaunchConfiguration(
        "depth_viz_show_grid_values")
    parameters["publish_vio_path"] = LaunchConfiguration("publish_vio_path")
    parameters["depth_pointcloud_frame_id"] = ParameterValue(
        LaunchConfiguration("depth_pointcloud_frame_id"), value_type=str)
    parameters["tf_mode"] = ParameterValue(LaunchConfiguration("tf_mode"), value_type=str)
    parameters["mounting_config"] = ParameterValue(LaunchConfiguration("mounting_config"), value_type=str)
    parameters["publish_mounting_tf"] = LaunchConfiguration("publish_mounting_tf")
    parameters["publish_odom_tf"] = LaunchConfiguration("publish_odom_tf")
    parameters["odom_frame"] = ParameterValue(LaunchConfiguration("odom_frame"), value_type=str)
    parameters["publish_vio_tf"] = LaunchConfiguration("publish_vio_tf")
    parameters["publish_sensor_tf"] = LaunchConfiguration("publish_sensor_tf")
    parameters["vio_path_max_poses"] = LaunchConfiguration("vio_path_max_poses")
    parameters["vio_path_publish_rate_hz"] = LaunchConfiguration(
        "vio_path_publish_rate_hz")
    return LaunchDescription([
        DeclareLaunchArgument("ipm_extent_m", default_value=ARGUMENT_DEFAULTS["ipm_extent_m"]),
        DeclareLaunchArgument("ipm_resolution_m_per_pixel", default_value=ARGUMENT_DEFAULTS["ipm_resolution_m_per_pixel"]),
        DeclareLaunchArgument("ipm_max_publish_rate_hz", default_value=ARGUMENT_DEFAULTS["ipm_max_publish_rate_hz"]),
        DeclareLaunchArgument("ipm_frame_id", default_value=ARGUMENT_DEFAULTS["ipm_frame_id"]),
        DeclareLaunchArgument("tf_mode", default_value=ARGUMENT_DEFAULTS["tf_mode"],
                              description="device or robot; robot requires mounting_config"),
        DeclareLaunchArgument("mounting_config", default_value=ARGUMENT_DEFAULTS["mounting_config"],
                              description="Robot config: URDF frame names or bridge-owned xyz/rpy"),
        DeclareLaunchArgument("publish_odom_tf", default_value=ARGUMENT_DEFAULTS["publish_odom_tf"]),
        DeclareLaunchArgument("odom_frame", default_value=ARGUMENT_DEFAULTS["odom_frame"]),
        DeclareLaunchArgument("publish_mounting_tf", default_value=ARGUMENT_DEFAULTS["publish_mounting_tf"],
                              description="Bridge owns installation TF; leave false when URDF owns it"),
        DeclareLaunchArgument(
            "depth_pointcloud_frame_id",
            default_value=ARGUMENT_DEFAULTS["depth_pointcloud_frame_id"],
            description="ROS_BODY device frame; empty uses orbvi_link; base_link restores legacy naming"),
        DeclareLaunchArgument(
            "publish_vio_tf",
            default_value=ARGUMENT_DEFAULTS["publish_vio_tf"]),
        DeclareLaunchArgument(
            "publish_sensor_tf",
            default_value=LaunchConfiguration("publish_vio_tf")),
        DeclareLaunchArgument(
            "sensor_data_best_effort",
            default_value=ARGUMENT_DEFAULTS["sensor_data_best_effort"]),
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
            parameters=[pano_config, parameters],
        ),
    ])
