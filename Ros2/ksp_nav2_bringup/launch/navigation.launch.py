import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ksp_nav2_bringup.launch_support import (
    adapter_nodes,
    grouped,
    localization_nodes,
    navigation_nodes,
    rviz_node,
)


def generate_launch_description():
    package_share = get_package_share_directory("ksp_nav2_bringup")
    default_nav2_params = os.path.join(package_share, "params", "nav2_params.yaml")

    source_scan_topic = LaunchConfiguration("scan_topic")
    nav_scan_topic = "/ksp_nav2/scan"
    map_file = LaunchConfiguration("map")
    nav2_params = LaunchConfiguration("nav2_params")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                description="Absolute path to a map YAML produced by map_saver_cli",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/ksp_vessel/lidar_2d/front_lidar/scan",
                description="LaserScan topic for the active vessel's 2D LiDAR",
            ),
            DeclareLaunchArgument("nav2_params", default_value=default_nav2_params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            grouped(
                adapter_nodes(nav2_params, use_sim_time, source_scan_topic)
                + localization_nodes(
                    nav2_params, use_sim_time, nav_scan_topic, map_file
                )
                + navigation_nodes(nav2_params, use_sim_time, nav_scan_topic)
                + [rviz_node(use_sim_time, use_rviz)]
            ),
        ]
    )
