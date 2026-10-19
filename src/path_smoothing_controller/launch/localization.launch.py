
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    package_share = get_package_share_directory(
        'path_smoothing_controller'
    )
    bringup_share = get_package_share_directory('robot_simulation')

    param_file = os.path.join(
        package_share, 'config', 'navigation_params.yaml'
    )
    default_map = os.path.join(
        bringup_share, 'maps', 'testbed_world.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'map_file',
            default_value=default_map,
            description='Path to the Testbed map YAML',
        ),

        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                param_file,
                {
                    'use_sim_time': use_sim_time,
                    'yaml_filename': LaunchConfiguration('map_file'),
                },
            ],
        ),

        # AMCL localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                param_file,
                {'use_sim_time': use_sim_time},
            ],
        ),

        # Manage both lifecycle nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
    ])
