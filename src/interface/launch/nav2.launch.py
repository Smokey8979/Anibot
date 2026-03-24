"""
nav2.launch.py

Full Nav2 stack with AMCL localization and map_server.
Requires a pre-built map (build one with slam_full.launch.py first).

Included by anibot_full.launch.py. To run standalone:
  ros2 launch interface nav2.launch.py map:=/full/path/to/my_map.yaml
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('interface')
    params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([

        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file'
        ),

        # --- MAP SERVER: serves the static map on /map topic ---
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_yaml}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),

        # --- AMCL: Monte Carlo localization (map -> odom -> base_footprint) ---
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),

        # --- NAVIGATION STACK ---
        Node(package='nav2_controller', executable='controller_server',
            name='controller_server', output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),

        Node(package='nav2_smoother', executable='smoother_server',
            name='smoother_server', output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),

        Node(package='nav2_planner', executable='planner_server',
            name='planner_server', output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),

        Node(package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),

        Node(package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),

        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
            name='waypoint_follower', output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),

        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),

        # --- LIFECYCLE MANAGER: boots all nav2 nodes in correct order ---
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',        # must be first
                    'amcl',              # must be second
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ],
            }]
        ),
    ])
