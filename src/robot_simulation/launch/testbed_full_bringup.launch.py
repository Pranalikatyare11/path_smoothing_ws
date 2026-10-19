import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('robot_simulation')
    rviz_config = os.path.join(package_share, 'rviz', 'full_bringup2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=rviz_config,
            description='Absolute path to the RViz configuration file',
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            package_share, 'launch', 'robot_description.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            package_share, 'launch', 'spawn_playground.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            package_share, 'launch', 'spawn_testbed.launch.py'))),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),
    ])
