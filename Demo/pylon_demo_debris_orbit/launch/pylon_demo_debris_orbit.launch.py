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
    value = value or "demo_vehicle"
    return "_" + value if value[0].isdigit() else value


def _launch_demo(context):
    guidance_overrides = {}
    controller_overrides = {}
    estimator_overrides = {}
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
    for name in ("lidar_sensor_id", "camera_sensor_id", "target_source", "lidar_frame"):
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
    for name in ("demo_instance_id", "vessel_topic_prefix", "lidar_sensor_id", "target_source"):
        if name in guidance_overrides:
            estimator_overrides[name] = guidance_overrides[name]
    target_topic = LaunchConfiguration("target_topic").perform(context).strip() or f"{prefix}/demos/debris_orbit/{instance}/target"
    guidance_overrides["target_topic"] = target_topic
    estimator_overrides["target_topic"] = target_topic
    radius = LaunchConfiguration("orbit_radius").perform(context).strip()
    if radius:
        guidance_overrides["orbit_radius"] = float(radius)
        estimator_overrides["orbit_radius"] = float(radius)
    navigation = f"{prefix}/demos/debris_orbit/{instance}/navigation"
    local_frame = "pylon_debris_inertial_" + instance
    for overrides in (guidance_overrides, estimator_overrides, controller_overrides):
        overrides["world_frame"] = local_frame
    for overrides in (guidance_overrides, controller_overrides):
        overrides["pose_topic"] = navigation + "/pose"
        overrides["twist_topic"] = navigation + "/twist"
    controller_overrides["body_twist_topic"] = navigation + "/twist_body"
    guidance_parameters = [LaunchConfiguration("config_file")]
    controller_parameters = [LaunchConfiguration("config_file")]
    if guidance_overrides:
        guidance_parameters.append(guidance_overrides)
    if controller_overrides:
        controller_parameters.append(controller_overrides)
    return [
        Node(package="rviz2", executable="rviz2", name="pylon_demo_debris_orbit_rviz", output="screen",
             arguments=["-d", os.path.join(get_package_share_directory("pylon_demo_debris_orbit"), "rviz", "pylon_demo_debris_orbit.rviz"),
                        "-f", "pylon_debris_view_" + instance],
             remappings=[(f"/pylon_demo_debris_orbit_view/{name}", f"{prefix}/demos/debris_orbit/{instance}/{name}")
                         for name in ("markers", "path", "points", "target_points")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
        Node(package="pylon_demo_debris_orbit", executable="target_estimator", name="pylon_debris_target_estimator",
             output="screen", parameters=[LaunchConfiguration("config_file"), estimator_overrides],
             condition=IfCondition(LaunchConfiguration("estimator_enabled"))),
        Node(
            package="pylon_demo_debris_orbit",
            executable="pylon_demo_debris_orbit_node",
            name="pylon_demo_debris_orbit",
            output="screen",
            parameters=guidance_parameters,
        ),
        Node(
            package="pylon_vehicle_control",
            executable="setpoint_controller",
            name="pylon_demo_debris_orbit_controller",
            output="screen",
            parameters=controller_parameters,
            condition=IfCondition(LaunchConfiguration("controller_enabled")),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("pylon_demo_debris_orbit"), "config", "pylon_demo_debris_orbit.yaml"
    )
    arguments = [
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("estimator_enabled", default_value="true"),
        DeclareLaunchArgument("orbit_radius", default_value=""),
        DeclareLaunchArgument("target_source", default_value="lidar_imu", choices=["lidar_imu"]),
        DeclareLaunchArgument("target_topic", default_value=""),
        DeclareLaunchArgument("lidar_frame", default_value=""),
        DeclareLaunchArgument("enabled", default_value=""),
        DeclareLaunchArgument("demo_instance_id", default_value="demo_vehicle"),
        DeclareLaunchArgument("controller_id", default_value="pylon_demo_debris_orbit_demo"),
        DeclareLaunchArgument("controller_enabled", default_value="true"),
        DeclareLaunchArgument("lidar_sensor_id", default_value=""),
        DeclareLaunchArgument("camera_sensor_id", default_value=""),
        DeclareLaunchArgument("vessel_topic_prefix", default_value="/ksp_vessel"),
        DeclareLaunchArgument("setpoint_topic", default_value=""),
        DeclareLaunchArgument("wrench_command_topic", default_value=""),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_demo)])
