"""Autonomy layer on the kinematic unicycle, no Gazebo. One vehicle v1 inside the demo corridor."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('corridor_nav')
    corridor = LaunchConfiguration('corridor')
    return LaunchDescription([
        DeclareLaunchArgument('corridor', default_value=os.path.join(share, 'config', 'corridor_demo.geojson')),
        Node(package='corridor_sim', executable='unicycle_sim', namespace='v1', name='unicycle_sim',
             parameters=[{'origin_lat': 52.7000, 'origin_lon': -3.0000, 'start_x': 20.0, 'start_y': 20.0,
                          'child_frame_id': 'v1/base_link'}]),
        Node(package='corridor_nav', executable='waypoint_follower', namespace='v1', name='waypoint_follower',
             parameters=[{'speed_mps': 3.0, 'lookahead_m': 3.0, 'arrive_tolerance_m': 1.5}]),
        Node(package='corridor_nav', executable='geofence_monitor', name='geofence_monitor',
             parameters=[{'corridor_geojson': corridor, 'vehicles': ['v1'], 'warn_margin_m': 15.0}]),
    ])
