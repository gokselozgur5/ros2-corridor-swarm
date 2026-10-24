from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
            parameters=[{'port': 8765, 'address': '0.0.0.0', 'send_buffer_limit': 100000000}],
            output='screen',
        ),
        Node(
            package='corridor_sim', executable='vehicle_sim', name='vehicle_sim',
            parameters=[{'name': 'vehicle', 'speed_mps': 20.0}],
            output='screen',
        ),
    ])
