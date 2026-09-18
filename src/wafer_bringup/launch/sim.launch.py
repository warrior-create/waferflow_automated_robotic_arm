#!/usr/bin/env python3
"""
sim.launch.py
=============
Launches the complete WaferFlow simulation using Ignition Gazebo 6 (Fortress).

Architecture (arm64 / aarch64):
  - ign_ros2_control/IgnitionSystem hardware plugin in URDF
  - Ignition Gazebo spawns the robot via ros_gz_sim
  - joint_state_broadcaster + scara_arm_controller spawned after Gazebo
  - robot_state_publisher publishes TF from URDF
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, ExecuteProcess, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description  = get_package_share_directory('wafer_description')
    pkg_moveit_config = get_package_share_directory('wafer_moveit_config')
    pkg_ros_gz_sim   = get_package_share_directory('ros_gz_sim')

    # ── Set IGN_GAZEBO_RESOURCE_PATH so models are found ────────────────
    models_path = os.path.join(pkg_description, 'models')
    set_gz_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_path
    )

    # ── Robot description (xacro → URDF string) ──────────────────────────
    xacro_file = os.path.join(pkg_description, 'urdf', 'scara_arm.xacro')
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # ── Robot State Publisher ────────────────────────────────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # ── Ignition Gazebo ──────────────────────────────────────────────────
    world_file = os.path.join(pkg_description, 'worlds', 'wafer_cell.world')
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items()
    )

    # ── Spawn robot into Ignition Gazebo ─────────────────────────────────
    # We use ros_gz_sim create service which reads /robot_description topic
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'scara_arm',
            '-topic', '/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
        ],
        output='screen'
    )

    # ── ros_gz_bridge: bridge /clock and camera topics ──────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/waferflow/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/waferflow/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/world/wafer_cell/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/world/wafer_cell/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/waferflow/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/waferflow/detach@std_msgs/msg/Empty]ignition.msgs.Empty'
        ],
        remappings=[
            ('/world/wafer_cell/pose/info', '/tf'),
            ('/world/wafer_cell/dynamic_pose/info', '/tf')
        ],
        output='screen'
    )

    # ── Controller spawners (delayed to let Ignition start) ──────────────
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    scara_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # ── ROSBridge Server (Web Dashboard) ─────────────────────────────────
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource,
        rsp_node,
        rosbridge_server,
        ign_gazebo,
        # Give Gazebo 3s to start before spawning the robot
        TimerAction(period=3.0, actions=[spawn_robot, clock_bridge]),
        # Spawn controllers 6s after launch (after robot is in Gazebo)
        TimerAction(period=6.0, actions=[joint_state_broadcaster]),
        TimerAction(period=8.0, actions=[scara_arm_controller]),
    ])
