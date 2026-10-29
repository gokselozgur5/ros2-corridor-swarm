"""Autonomy layer on kinematic unicycles, no Gazebo. `n` vehicles v1..vn inside the demo corridor,
each with a follower and a patrol server; one geofence monitor, one fleet_state and, unless overlay:=false,
one map_overlay (corridor, routes and heading lines for the Foxglove map) for all (issues #10, #20, #19)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    share = get_package_share_directory('corridor_nav')
    corridor = LaunchConfiguration('corridor').perform(context)
    n = int(LaunchConfiguration('n').perform(context))
    first = int(LaunchConfiguration('first').perform(context))
    lookahead = float(LaunchConfiguration('lookahead_m').perform(context))
    speed = float(LaunchConfiguration('speed_mps').perform(context))
    overlay = LaunchConfiguration('overlay').perform(context).lower() == 'true'
    names = [f'v{i}' for i in range(first, first + n)]
    nodes = []
    for i, v in enumerate(names):
        nodes += [
            Node(package='corridor_sim', executable='unicycle_sim', namespace=v, name='unicycle_sim',
                 parameters=[{'origin_lat': 52.7000, 'origin_lon': -3.0000,
                              'start_x': 20.0 + 25.0 * (first - 1 + i), 'start_y': 20.0, 'child_frame_id': f'{v}/base_link'}]),
            Node(package='corridor_nav', executable='waypoint_follower', namespace=v, name='waypoint_follower',
                 parameters=[{'speed_mps': speed, 'lookahead_m': lookahead, 'arrive_tolerance_m': 1.5, 'corridor_geojson': corridor}]),
            Node(package='corridor_nav', executable='patrol_server', namespace=v, name='patrol_server',
                 parameters=[{'corridor_geojson': corridor, 'vehicle': v, 'inset_m': 30.0 + 8.0 * i}]),
        ]
    nodes.append(Node(package='corridor_nav', executable='geofence_monitor', name='geofence_monitor',
                      parameters=[{'corridor_geojson': corridor, 'vehicles': names, 'warn_margin_m': 15.0}]))
    nodes.append(Node(package='corridor_nav', executable='fleet_state', name='fleet_state',
                      parameters=[{'corridor_geojson': corridor, 'vehicles': names}]))
    if overlay:   # the routes it draws are the patrol servers' insets, 30 m plus 8 m per vehicle
        nodes.append(Node(package='corridor_nav', executable='map_overlay', name='map_overlay',
                          parameters=[{'corridor_geojson': corridor, 'vehicles': names, 'insets_m': [30.0 + 8.0 * i for i in range(len(names))]}]))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('corridor_nav')
    return LaunchDescription([
        DeclareLaunchArgument('corridor', default_value=os.path.join(share, 'config', 'corridor_demo.geojson')),
        DeclareLaunchArgument('n', default_value='1'),
        DeclareLaunchArgument('first', default_value='1'),   # first vehicle number, e.g. first:=2 n:=2 beside a Gazebo v1
        DeclareLaunchArgument('lookahead_m', default_value='3.0'),
        DeclareLaunchArgument('speed_mps', default_value='3.0'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),   # another bridge may hold 8765 on a shared host
        DeclareLaunchArgument('overlay', default_value='true'),   # map_overlay needs foxglove_msgs (apt ros-jazzy-foxglove-msgs or steps/15)
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
             condition=IfCondition(LaunchConfiguration('foxglove')),
             parameters=[{'port': ParameterValue(LaunchConfiguration('foxglove_port'), value_type=int), 'address': '0.0.0.0', 'send_buffer_limit': 100000000}]),
        OpaqueFunction(function=launch_setup),
    ])
