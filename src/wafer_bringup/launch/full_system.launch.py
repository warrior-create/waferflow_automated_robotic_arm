import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('wafer_bringup')
    pkg_description = get_package_share_directory('wafer_description')
    
    sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'sim.launch.py'))
    )

    moveit_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'moveit_bringup.launch.py'))
    )

    vision_node = Node(
        package='wafer_vision',
        executable='alignment_service',
        output='screen'
    )

    trajectory_node = Node(
        package='wafer_trajectory',
        executable='trajectory_action_server',
        output='screen'
    )

    control_node = Node(
        package='wafer_control',
        executable='pick_place_fsm',
        output='screen'
    )

    xacro_file = os.path.join(pkg_description, 'urdf', 'scara_arm.xacro')
    from launch.substitutions import Command
    from launch_ros.parameter_descriptions import ParameterValue
    robot_description = {'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)}

    pkg_moveit_config = get_package_share_directory('wafer_moveit_config')
    srdf_file = os.path.join(pkg_moveit_config, 'config', 'scara_arm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_description, 'rviz', 'wafer_view.rviz')],
        parameters=[robot_description, robot_description_semantic],
        output='screen'
    )

    return LaunchDescription([
        sim_cmd,
        moveit_cmd,
        vision_node,
        trajectory_node,
        control_node,
        rviz_node
    ])
