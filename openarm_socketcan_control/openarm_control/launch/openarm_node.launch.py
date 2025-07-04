#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Parameters
    arm_namespace = LaunchConfiguration('arm_namespace')
    can_interface = LaunchConfiguration('can_interface')
    config_file = LaunchConfiguration('config_file')

    # Get the config file path
    config_file_path = PathJoinSubstitution([
        FindPackageShare('openarm_socketcan_ros2'),
        'config',
        config_file
    ])

    # OpenArm node
    openarm_node = Node(
        package='openarm_socketcan_ros2',
        executable='openarm_socketcan_node',
        name='openarm_node',
        namespace=arm_namespace,
        parameters=[config_file_path],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )

    return [openarm_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_namespace',
            default_value='openarm',
            description='Namespace for the arm'
        ),
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface to use'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='openarm_composed_params.yaml',
            description='Configuration file to use'
        ),
        OpaqueFunction(function=launch_setup)
    ])
