"""
anibot_full.launch.py

ONE COMMAND for autonomous navigation with a pre-built map:
  - bringup (robot_state_publisher + serial odom bridge)
  - RPLIDAR A2M8 launch
  - Nav2 (map_server + AMCL + full navigation stack)

USAGE:
  ros2 launch interface anibot_full.launch.py map:=/home/pi/maps/my_map.yaml

For SLAM (building a new map) use slam_full.launch.py:
  ros2 launch interface slam_full.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    interface_pkg = get_package_share_directory('interface')
    rplidar_pkg   = get_package_share_directory('rplidar_ros')

    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([

        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file (e.g. /home/pi/maps/my_map.yaml)'
        ),

        # 1. Robot bringup: robot_state_publisher + serial odom bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(interface_pkg, 'launch', 'bringup.launch.py')
            )
        ),

        # 2. RPLIDAR (A1M8 confirmed to work with a2m8 launch)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rplidar_pkg, 'launch', 'rplidar_a2m8_launch.py')
            )
        ),

        # 3. Nav2: map_server + AMCL + full navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(interface_pkg, 'launch', 'nav2.launch.py')
            ),
            launch_arguments={'map': map_yaml}.items()
        ),

    ])
