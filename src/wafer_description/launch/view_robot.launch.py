#!/usr/bin/env python3
"""
view_robot.launch.py
====================
Launch file for Phase 1: static URDF viewing in RViz without Gazebo.
- robot_state_publisher: publishes TF from URDF
- joint_state_publisher_gui: slider GUI to move joints manually
- rviz2: visualization with the saved config

Usage:
    ros2 launch wafer_description view_robot.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('wafer_descrifption')

    # ── Arguments ─────────────────────────────────────────────────────
    use_gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='true',
        description='Launch joint_state_publisher_gui (true) or static publisher (false)'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_description, 'rviz', 'wafer_view.rviz'),
        description='Path to RViz config file'
    )

    use_gui = LaunchConfiguration('use_gui')
    rviz_config = LaunchConfiguration('rviz_config')

    # ── Robot description from xacro ──────────────────────────────────
    xacro_file = os.path.join(pkg_description, 'urdf', 'scara_arm.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # ── Nodes ─────────────────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=None  # always launch for view_robot
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        use_gui_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
