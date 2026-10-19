import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('path_smoothing_controller')
    params = os.path.join(package_share, 'config', 'navigation_params.yaml')
    local_costmap_params = os.path.join(
        package_share, 'config', 'local_costmap_params.yaml')
    controller_params = os.path.join(
        package_share, 'config', 'trajectory_controller_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('goal_topic', default_value='/trajectory_goal'),
        DeclareLaunchArgument('waypoint_start_topic', default_value='/start_waypoint_list'),
        DeclareLaunchArgument('waypoint_list_enabled', default_value='True'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            output='screen',
            parameters=[local_costmap_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_custom_local_costmap',
            namespace='costmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['costmap'],
                # The standalone costmap executable does not create a Nav2 lifecycle bond.
                'bond_timeout': 0.0,
            }],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_custom_planner',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['planner_server'],
            }],
        ),
        Node(
            package='path_smoothing_controller',
            executable='trajectory_controller',
            name='trajectory_controller',
            output='screen',
            parameters=[controller_params, {
                    'use_sim_time': use_sim_time,
                    'goal_topic': LaunchConfiguration('goal_topic'),
                    'waypoint_start_topic': LaunchConfiguration('waypoint_start_topic'),
                    'waypoint_list_enabled': LaunchConfiguration('waypoint_list_enabled'),
                    'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                }],
        ),
    ])
