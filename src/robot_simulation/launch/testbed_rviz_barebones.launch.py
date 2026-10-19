import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('robot_simulation')
    robot_description = Command([
        'xacro ',
        os.path.join(package_share, 'urdf', 'testbed.xacro'),
    ])
    rviz_config = os.path.join(package_share, 'rviz', 'testbed_barebones.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_node',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description,
            }],
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config],
        ),
    ])
