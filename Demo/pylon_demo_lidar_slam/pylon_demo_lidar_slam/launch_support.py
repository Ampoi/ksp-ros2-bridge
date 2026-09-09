"""Factories shared by the mapping and saved-map launch descriptions."""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch_ros.actions import Node


def adapter_nodes(parameters_file, use_sim_time, source_scan_topic):
    """Create the KSP-specific scan odometry and wrench adapter nodes."""
    common_parameters = [parameters_file, {"use_sim_time": use_sim_time}]
    return [
        Node(
            package="pylon_demo_lidar_slam",
            executable="laser_scan_odometry",
            name="laser_scan_odometry",
            output="screen",
            parameters=common_parameters + [{"scan_topic": source_scan_topic}],
        ),
        Node(
            package="pylon_demo_lidar_slam",
            executable="planar_wrench_controller",
            name="planar_wrench_controller",
            output="screen",
            parameters=common_parameters,
        ),
    ]


def navigation_nodes(parameters_file, use_sim_time, scan_topic):
    """Create the minimal Nav2 navigation servers needed by NavigateToPose."""
    parameters = [parameters_file, {"use_sim_time": use_sim_time}]
    scan_remapping = [("scan", scan_topic)]
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]
    return [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=parameters,
            remappings=scan_remapping,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=parameters,
            remappings=scan_remapping,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=parameters,
            remappings=scan_remapping,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=parameters,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": True},
                {"node_names": lifecycle_nodes},
            ],
        ),
    ]


def localization_nodes(parameters_file, use_sim_time, scan_topic, map_file):
    """Create saved-map localization nodes and their lifecycle manager."""
    parameters = [parameters_file, {"use_sim_time": use_sim_time}]
    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=parameters + [{"yaml_filename": map_file}],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=parameters,
            remappings=[("scan", scan_topic)],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": True},
                {"node_names": ["map_server", "amcl"]},
            ],
        ),
    ]


def rviz_node(use_sim_time, enabled):
    """Create RViz with the standard Nav2 panel and goal tool configuration."""
    rviz_config = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "rviz",
        "nav2_default_view.rviz",
    )
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("/scan", "/pylon/lidar_slam/scan"),
                    ("/odom", "/pylon/lidar_slam/odom")],
        condition=IfCondition(enabled),
    )


def grouped(nodes):
    """Keep this stack visibly grouped in launch introspection output."""
    return GroupAction(actions=nodes)
