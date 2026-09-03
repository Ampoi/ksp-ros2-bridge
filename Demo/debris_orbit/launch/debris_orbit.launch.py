"""Launch the debris orbit demo from its installed YAML configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import re


def _topic_component(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_]+", "_", value).strip("_").lower()
    return value or "demo_vehicle"


def _launch_demo(context):
    guidance_overrides = {}
    controller_overrides = {}
    enabled = LaunchConfiguration("enabled").perform(context).strip().lower()
    if enabled:
        if enabled not in ("true", "false"):
            raise ValueError("enabled must be true or false")
        guidance_overrides["enabled"] = enabled == "true"
    for name in ("demo_instance_id", "vessel_topic_prefix"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            guidance_overrides[name] = value
            if name == "vessel_topic_prefix":
                controller_overrides[name] = value
    for name in ("lidar_sensor_id", "camera_sensor_id"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            guidance_overrides[name] = value
    prefix = LaunchConfiguration("vessel_topic_prefix").perform(context).strip().rstrip("/")
    instance = _topic_component(
        LaunchConfiguration("demo_instance_id").perform(context).strip()
    )
    setpoint_topic = LaunchConfiguration("setpoint_topic").perform(context).strip()
    setpoint_topic = setpoint_topic or f"{prefix}/demos/debris_orbit/{instance}/setpoint"
    guidance_overrides["setpoint_topic"] = setpoint_topic
    controller_overrides["setpoint_topic"] = setpoint_topic
    controller_overrides["controller_status_topic"] = (
        f"{prefix}/demos/debris_orbit/{instance}/controller_status"
    )
    controller_overrides["controller_id"] = LaunchConfiguration(
        "controller_id"
    ).perform(context).strip()
    wrench_command_topic = LaunchConfiguration("wrench_command_topic").perform(context).strip()
    if wrench_command_topic:
        controller_overrides["wrench_command_topic"] = wrench_command_topic
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
            package="ksp_vehicle_control",
            executable="setpoint_controller",
            name="debris_orbit_controller",
            output="screen",
            parameters=controller_parameters,
            condition=IfCondition(LaunchConfiguration("controller_enabled")),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("debris_orbit"), "config", "debris_orbit.yaml"
    )
    arguments = [
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("enabled", default_value=""),
        DeclareLaunchArgument("demo_instance_id", default_value="demo_vehicle"),
        DeclareLaunchArgument("controller_id", default_value="debris_orbit_demo"),
        DeclareLaunchArgument("controller_enabled", default_value="true"),
        DeclareLaunchArgument("lidar_sensor_id", default_value=""),
        DeclareLaunchArgument("camera_sensor_id", default_value=""),
        DeclareLaunchArgument("vessel_topic_prefix", default_value="/ksp_vessel"),
        DeclareLaunchArgument("setpoint_topic", default_value=""),
        DeclareLaunchArgument("wrench_command_topic", default_value=""),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_demo)])
