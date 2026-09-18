#!/usr/bin/env python3
"""
move_group.launch.py
====================
Launches the MoveIt2 move_group node with all required parameters.
This is Phase 3 of the build plan — get basic MoveIt planning working.

Usage:
    ros2 launch wafer_moveit_config move_group.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def generate_launch_description():
    pkg_description = get_package_share_directory('wafer_description')
    pkg_moveit     = get_package_share_directory('wafer_moveit_config')

    # ── Robot Description ──────────────────────────────────────────────
    xacro_file = os.path.join(pkg_description, 'urdf', 'scara_arm.xacro')
    robot_description = {'robot_description': ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )}

    srdf_file = os.path.join(pkg_moveit, 'config', 'scara_arm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml = os.path.join(pkg_moveit, 'config', 'kinematics.yaml')
    joint_limits_yaml = os.path.join(pkg_moveit, 'config', 'joint_limits.yaml')
    ompl_yaml = os.path.join(pkg_moveit, 'config', 'ompl_planning.yaml')

    # Load moveit_controllers.yaml as a dict so top-level keys
    # (moveit_controller_manager, etc.) are passed as node parameters
    # rather than via --params-file (which requires ros__parameters: wrapping).
    controllers_yaml_path = os.path.join(pkg_moveit, 'config', 'moveit_controllers.yaml')
    with open(controllers_yaml_path, 'r') as f:
        moveit_controllers = yaml.safe_load(f)
    # Only keep MoveIt-relevant keys (drop controller_manager / spawner config)
    moveit_controller_params = {
        k: v for k, v in moveit_controllers.items()
        if k in ('moveit_controller_manager', 'moveit_simple_controller_manager')
    }

    # ── move_group node ────────────────────────────────────────────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'robot_description_planning': {'joint_limits': joint_limits_yaml}},
            ompl_yaml,
            moveit_controller_params,
            {
                'planning_scene_monitor_options': {
                    'name': 'planning_scene_monitor',
                    'robot_description': 'robot_description',
                    'joint_state_topic': '/joint_states',
                    'attached_collision_object_topic': '/move_group/planning_scene_monitor',
                    'publish_planning_scene_topic': '/move_group/publish_planning_scene',
                    'monitored_planning_scene_topic': '/move_group/monitored_planning_scene',
                    'wait_for_initial_state_timeout': 10.0,
                },
                'use_sim_time': True,
            },
        ],
    )

    return LaunchDescription([move_group_node])
