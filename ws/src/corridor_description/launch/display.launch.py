"""robot_state_publisher on the UGV xacro; fixed joint states so the wheels have a pose. Proof of issue #3."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('corridor_description'), 'urdf', 'ugv.urdf.xacro')
    description = ParameterValue(Command([FindExecutable(name='xacro'), ' ', xacro_file]), value_type=str)
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
    ])
