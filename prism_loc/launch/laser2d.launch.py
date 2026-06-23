import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('prism_loc')
    params = os.path.join(pkg, 'params', 'laser2d.yaml')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='path to map .yaml for map_server'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['map_server']}]),
        Node(package='prism_loc', executable='prism_loc_node_main', name='prism_loc',
             output='screen', parameters=[params, {'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             condition=IfCondition(use_rviz),
             arguments=['-d', os.path.join(pkg, 'rviz', 'prism_loc.rviz')]),
    ])
