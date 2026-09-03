"""Launch the debris orbit demo from its installed YAML configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _launch_demo(context):
    overrides = {}
    enabled = LaunchConfiguration("enabled").perform(context).strip().lower()
    if enabled:
        if enabled not in ("true", "false"):
            raise ValueError("enabled must be true or false")
        overrides["enabled"] = enabled == "true"
    for name in ("platform_id", "lidar_sensor_id", "camera_sensor_id"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            overrides[name] = value
    parameters = [LaunchConfiguration("config_file")]
    if overrides:
        parameters.append(overrides)
    return [
        Node(
            package="debris_orbit",
            executable="debris_orbit_node",
            name="debris_orbit",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("debris_orbit"), "config", "debris_orbit.yaml"
    )
    arguments = [
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("enabled", default_value=""),
        DeclareLaunchArgument("platform_id", default_value=""),
        DeclareLaunchArgument("lidar_sensor_id", default_value=""),
        DeclareLaunchArgument("camera_sensor_id", default_value=""),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_demo)])
