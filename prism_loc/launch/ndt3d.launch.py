import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('prism_loc')
    params = os.path.join(pkg, 'params', 'ndt3d.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map_pcd_path', description='path to prior map .pcd'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='prism_loc', executable='prism_loc_node_main', name='prism_loc',
             output='screen',
             parameters=[params,
                         {'map_pcd_path': LaunchConfiguration('map_pcd_path'),
                          'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(pkg, 'rviz', 'prism_loc.rviz')]),
    ])
