"""Launch the debris orbit demo from its installed YAML configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _launch_demo(context):
    guidance_overrides = {}
    controller_overrides = {}
    enabled = LaunchConfiguration("enabled").perform(context).strip().lower()
    if enabled:
        if enabled not in ("true", "false"):
            raise ValueError("enabled must be true or false")
        guidance_overrides["enabled"] = enabled == "true"
    for name in ("platform_id", "vessel_topic_prefix"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            guidance_overrides[name] = value
            controller_overrides[name] = value
    for name in ("lidar_sensor_id", "camera_sensor_id"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            guidance_overrides[name] = value
    body_wrench_topic = LaunchConfiguration("body_wrench_topic").perform(context).strip()
    if body_wrench_topic:
        controller_overrides["body_wrench_topic"] = body_wrench_topic
    guidance_parameters = [LaunchConfiguration("config_file")]
    controller_parameters = [LaunchConfiguration("config_file")]
    if guidance_overrides:
        guidance_parameters.append(guidance_overrides)
    if controller_overrides:
        controller_parameters.append(controller_overrides)
    return [
        Node(
            package="debris_orbit",
            executable="debris_orbit_node",
            name="debris_orbit",
            output="screen",
            parameters=guidance_parameters,
        ),
        Node(
            package="debris_orbit",
            executable="debris_orbit_controller",
            name="debris_orbit_controller",
            output="screen",
            parameters=controller_parameters,
        ),
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
        DeclareLaunchArgument("vessel_topic_prefix", default_value=""),
        DeclareLaunchArgument("body_wrench_topic", default_value=""),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_demo)])
