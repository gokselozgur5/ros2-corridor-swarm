"""Headless Gazebo with the corridor world, one UGV (v1) spawned from the xacro, the bridge, robot_state_publisher.

Issue #4 proof: ros2 topic hz on /v1/scan /v1/imu /v1/fix /v1/camera/image under sim time.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gz_share = get_package_share_directory('corridor_gz')
    desc_share = get_package_share_directory('corridor_description')
    world = LaunchConfiguration('world')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    xacro_file = os.path.join(desc_share, 'urdf', 'ugv.urdf.xacro')
    description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' sim:=true ns:=v1 prefix:=v1/ use_ros2_control:=', use_ros2_control,
        ' controllers_yaml:=', os.path.join(gz_share, 'config', 'controllers_v1.yaml')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(gz_share, 'worlds', 'corridor.sdf')),
        DeclareLaunchArgument('use_ros2_control', default_value='false'),
        ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '-v', '2', world], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='v1',
             parameters=[{'robot_description': description, 'use_sim_time': True, 'frame_prefix': ''}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'corridor', '-name', 'v1', '-topic', '/v1/robot_description',
                        '-x', '20', '-y', '20', '-z', '0.5']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(gz_share, 'config', 'bridge_v1.yaml'), 'use_sim_time': True}]),
        # ros2_control: spawners wait for /v1/controller_manager, which the gz plugin starts with the model.
        Node(package='controller_manager', executable='spawner', output='screen', condition=IfCondition(use_ros2_control),
             arguments=['joint_state_broadcaster', '-c', '/v1/controller_manager', '--controller-manager-timeout', '60']),
        Node(package='controller_manager', executable='spawner', output='screen', condition=IfCondition(use_ros2_control),
             arguments=['diff_drive', '-c', '/v1/controller_manager', '--controller-manager-timeout', '60',
                        '--controller-ros-args', '-r /v1/diff_drive/odom:=/v1/odom']),
        Node(package='corridor_nav', executable='twist_stamper', namespace='v1', name='twist_stamper',
             condition=IfCondition(use_ros2_control), parameters=[{'frame_id': 'v1/base_footprint', 'use_sim_time': True}]),
    ])
