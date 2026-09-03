import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

from ksp_nav2_bringup.launch_support import (
    adapter_nodes,
    grouped,
    navigation_nodes,
    rviz_node,
)


def generate_launch_description():
    package_share = get_package_share_directory("ksp_nav2_bringup")
    default_nav2_params = os.path.join(package_share, "params", "nav2_params.yaml")
    default_slam_params = os.path.join(package_share, "params", "slam_toolbox.yaml")

    source_scan_topic = LaunchConfiguration("scan_topic")
    nav_scan_topic = "/ksp_nav2/scan"
    nav2_params = LaunchConfiguration("nav2_params")
    slam_params = LaunchConfiguration("slam_params")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    slam = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            slam_params,
            {"use_sim_time": use_sim_time},
            {"use_lifecycle_manager": True},
        ],
        remappings=[("scan", nav_scan_topic)],
    )
    slam_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["slam_toolbox"]},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/ksp_vessel/lidar_2d/front_lidar/scan",
                description="LaserScan topic for the active vessel's 2D LiDAR",
            ),
            DeclareLaunchArgument("nav2_params", default_value=default_nav2_params),
            DeclareLaunchArgument("slam_params", default_value=default_slam_params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            grouped(
                adapter_nodes(nav2_params, use_sim_time, source_scan_topic)
                + [slam, slam_lifecycle_manager]
                + navigation_nodes(nav2_params, use_sim_time, nav_scan_topic)
                + [rviz_node(use_sim_time, use_rviz)]
            ),
        ]
    )
