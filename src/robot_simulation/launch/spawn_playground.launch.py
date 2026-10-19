#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_simulation = get_package_share_directory('robot_simulation')

    install_dir = get_package_prefix('robot_simulation')

    gazebo_models_path = os.path.join(pkg_robot_simulation, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = (
            os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir
            + '/share:' + gazebo_models_path)
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + '/share:' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = (
            os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib')
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print('GAZEBO MODELS PATH==' + os.environ['GAZEBO_MODEL_PATH'])
    print('GAZEBO PLUGINS PATH==' + os.environ['GAZEBO_PLUGIN_PATH'])

    world_path = os.path.join(
        pkg_robot_simulation, 'worlds', 'testbed_playground.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='SDF world file',
        ),
        gazebo,
    ])
