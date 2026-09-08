import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    share=get_package_share_directory('mun_rover_demo')
    params=LaunchConfiguration('params_file')
    servers=[('nav2_controller','controller_server'),('nav2_planner','planner_server'),('nav2_bt_navigator','bt_navigator')]
    nodes=[DeclareLaunchArgument('params_file',default_value=os.path.join(share,'config','nav2.yaml')),
           DeclareLaunchArgument('bt_xml',default_value=os.path.join(share,'config','navigate.xml')),
           DeclareLaunchArgument('lidar_sensor_id',default_value='front_lidar'),
           DeclareLaunchArgument('use_rviz',default_value='true'),
           DeclareLaunchArgument('start_bridge',default_value='true'),
           DeclareLaunchArgument('evaluate',default_value='false')]
    # Ground truth publishers are absent in the default navigation launch.
    for evaluation in (False,True):
        args=[] if evaluation else ['--disable-ground-truth']
        # evaluate launches an independent observer; runtime never subscribes truth.
        from launch.conditions import IfCondition
        from launch.substitutions import PythonExpression
        condition=IfCondition(PythonExpression(["'",LaunchConfiguration('start_bridge'),"' == 'true' and '",LaunchConfiguration('evaluate'),"' == '",str(evaluation).lower(),"'"]))
        nodes.append(Node(package='ksp_lidar_bridge',executable='udp_bridge',arguments=args,condition=condition,output='screen'))
    nodes.append(Node(package='mun_rover_demo',executable='rover_node',output='screen',
        parameters=[{'lidar_sensor_id':LaunchConfiguration('lidar_sensor_id')}]))
    for package,executable in servers:
        overrides={'use_sim_time':False}
        if executable=='bt_navigator': overrides['default_nav_to_pose_bt_xml']=LaunchConfiguration('bt_xml')
        nodes.append(Node(package=package,executable=executable,name=executable,namespace='mun_rover',output='screen',
            parameters=[params,overrides],remappings=[('/tf','/tf'),('/tf_static','/tf_static')]))
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',namespace='mun_rover',
        parameters=[{'autostart':True,'node_names':[e for _,e in servers]}],output='screen'))
    nodes.append(Node(package='mun_rover_demo',executable='evaluate',condition=IfCondition(LaunchConfiguration('evaluate')),output='screen'))
    nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',os.path.join(share,'rviz','mun_rover.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),output='screen'))
    return LaunchDescription(nodes)
