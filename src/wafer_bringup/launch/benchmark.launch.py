import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('wafer_bringup')
    
    full_system_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'full_system.launch.py'))
    )

    metrics_logger_node = Node(
        package='wafer_benchmark',
        executable='metrics_logger',
        output='screen'
    )

    sweep_runner_node = Node(
        package='wafer_benchmark',
        executable='sweep_runner',
        output='screen'
    )

    return LaunchDescription([
        full_system_cmd,
        metrics_logger_node,
        sweep_runner_node
    ])
