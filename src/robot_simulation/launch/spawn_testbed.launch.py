from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'testbed',
            '-x', '0.0', '-y', '5.0', '-z', '0.0',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
            '-topic', '/robot_description',
        ],
    )
    return LaunchDescription([spawn_robot])
