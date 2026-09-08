"""Launch the 3D LiDAR position estimator demo from its YAML configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _launch_demo(context):
    overrides = {}
    for name in ("lidar_sensor_id", "vessel_topic_prefix", "odom_frame", "base_frame"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            overrides[name] = value
    for name in ("lidar_topic", "odom_topic", "status_topic", "error_topic", "pose_topic"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            overrides[name] = value
    parameters = [LaunchConfiguration("config_file")]
    if overrides:
        parameters.append(overrides)
    return [
        Node(
            package="pylon_demo_position_estimator",
            executable="pylon_demo_position_estimator_node",
            name="pylon_demo_position_estimator",
            output="screen",
            parameters=parameters,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("pylon_demo_position_estimator"),
        "config",
        "pylon_demo_position_estimator.yaml",
    )
    arguments = [
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("lidar_sensor_id", default_value=""),
        DeclareLaunchArgument("vessel_topic_prefix", default_value=""),
        DeclareLaunchArgument("lidar_topic", default_value=""),
        DeclareLaunchArgument("odom_topic", default_value=""),
        DeclareLaunchArgument("status_topic", default_value=""),
        DeclareLaunchArgument("error_topic", default_value=""),
        DeclareLaunchArgument("pose_topic", default_value=""),
        DeclareLaunchArgument("odom_frame", default_value=""),
        DeclareLaunchArgument("base_frame", default_value=""),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_demo)])
