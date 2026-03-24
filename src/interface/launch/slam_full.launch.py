"""
slam_full.launch.py

ONE COMMAND to start a full SLAM mapping session:
  - bringup (robot_state_publisher + serial odom bridge)
  - RPLIDAR A2M8 launch
  - slam_toolbox in async mapping mode

USAGE:
  ros2 launch interface slam_full.launch.py

After mapping, save your map:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

Then for autonomous nav:
  ros2 launch interface anibot_full.launch.py map:=$HOME/maps/my_map.yaml
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    interface_pkg = get_package_share_directory('interface')
    rplidar_pkg   = get_package_share_directory('rplidar_ros')

    slam_params = {
        'use_sim_time': False,
        'use_lifecycle_manager': False,   # CRITICAL: no lifecycle manager in our launch
        'solver_plugin': 'solver_plugins::CeresSolver',
        'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
        'ceres_preconditioner': 'SCHUR_JACOBI',
        'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
        'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
        'ceres_loss_function': 'None',
        'odom_frame': 'odom',
        'map_frame': 'map',
        'base_frame': 'base_footprint',
        'scan_topic': 'scan',
        'use_map_saver': False,
        'mode': 'mapping',
        'debug_logging': False,
        'throttle_scans': 1,
        'transform_publish_period': 0.02,
        'map_update_interval': 5.0,
        'resolution': 0.05,
        'max_laser_range': 8.0,       # A1M8 max reliable range is ~8m (not 12)
        'minimum_time_interval': 0.5,
        'transform_timeout': 0.2,
        'tf_buffer_duration': 30.0,
        'stack_size_to_use': 40000000,
        'enable_interactive_mode': True,
    }

    return LaunchDescription([

        # 1. Robot bringup: robot_state_publisher + serial odom bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(interface_pkg, 'launch', 'bringup.launch.py')
            )
        ),

        # 2. RPLIDAR A1M8 (using a2m8 launch as confirmed working)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rplidar_pkg, 'launch', 'rplidar_a2m8_launch.py')
            )
        ),

        # 3. slam_toolbox async mapping
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
        ),

        # 4. Lifecycle manager — auto configure + activate slam_toolbox on launch
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'bond_timeout': 0.0,   # disable bond heartbeat - slam_toolbox is slow to respond
                'node_names': ['slam_toolbox'],
            }]
        ),
    ])
