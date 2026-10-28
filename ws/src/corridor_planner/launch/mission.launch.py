"""Mission layer on top of a running vehicle layer (kinematic.launch.py or corridor_gz sim.launch.py):
patrol_server per vehicle, mission_planner (Opus), status_loop (Haiku). ANTHROPIC_API_KEY from the shell."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    corridor = LaunchConfiguration('corridor')
    dry_run = LaunchConfiguration('dry_run')
    status = LaunchConfiguration('status_loop')
    default_corridor = os.path.join(get_package_share_directory('corridor_nav'), 'config', 'corridor_demo.geojson')
    return LaunchDescription([
        DeclareLaunchArgument('corridor', default_value=default_corridor),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('status_loop', default_value='false'),
        DeclareLaunchArgument('start_patrol', default_value='true'),   # false when the vehicle layer already runs one
        DeclareLaunchArgument('fleet_state', default_value='true'),    # false when the vehicle layer already runs one (kinematic.launch.py)
        Node(package='corridor_nav', executable='fleet_state', name='fleet_state',
             condition=IfCondition(LaunchConfiguration('fleet_state')),
             parameters=[{'corridor_geojson': corridor, 'vehicles': ['v1']}]),
        Node(package='corridor_nav', executable='patrol_server', namespace='v1', name='patrol_server',
             condition=IfCondition(LaunchConfiguration('start_patrol')),
             parameters=[{'corridor_geojson': corridor, 'vehicle': 'v1', 'inset_m': 30.0}]),
        Node(package='corridor_planner', executable='mission_planner', name='mission_planner', output='screen',
             parameters=[{'corridor_geojson': corridor, 'vehicles': ['v1'], 'dry_run': dry_run}]),
        Node(package='corridor_planner', executable='status_loop', name='status_loop', output='screen',
             condition=IfCondition(status), parameters=[{'vehicles': ['v1'], 'period_s': 30.0}]),
    ])
