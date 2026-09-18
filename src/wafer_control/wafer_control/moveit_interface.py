#!/usr/bin/env python3
"""
moveit_interface.py
====================
Phase 6: Thin wrapper around MoveIt2's Python MoveGroupInterface.

Provides clean methods for:
  - Planning to named targets (e.g., 'home', 'approach_cassette')
  - Planning to joint-space targets (dict of joint_name → value)
  - Planning to Cartesian poses (geometry_msgs/Pose)
  - Extracting joint waypoints from a planned RobotTrajectory

This abstraction isolates wafer_control from MoveIt's API details,
making it easier to swap out the planner or mock it in tests.

TODO (Phase 6):
  - Initialize MoveGroupCommander correctly once MoveIt is running
  - Implement Cartesian planning via compute_cartesian_path
  - Add scene object management (collision boxes for cassette/stocker)
"""

import rclpy
from rclpy.node import Node
import numpy as np
from typing import Optional

# MoveIt Python bindings (moveit_commander in ROS1, moveit_py in ROS2 Humble)
# In Humble, use the MoveIt2 Python bindings via moveit_py or pymoveit2
try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False


class MoveItInterface:
    """
    Thin wrapper around MoveIt2 Python API for the WaferFlow SCARA arm.

    Used by PickPlaceFSM to request joint-space and Cartesian plans.
    """

    PLANNING_GROUP = 'scara_arm'
    TCP_LINK = 'tcp'

    def __init__(self, node: Node):
        """
        Initialize MoveIt interface.

        Args:
            node: the ROS2 node to use for logging and parameters
        """
        self._node = node
        self._moveit: Optional[object] = None
        self._arm = None
        self._robot_state = None

        if MOVEIT_AVAILABLE:
            self._init_moveit()
        else:
            node.get_logger().warn(
                'moveit_py not available — MoveItInterface running in STUB mode. '
                'All planning calls will return placeholder data.'
            )

    def _init_moveit(self) -> None:
        """
        Initialize MoveItPy and MoveGroup for the SCARA arm.

        TODO (Phase 6):
          from moveit.planning import MoveItPy
          self._moveit = MoveItPy(node_name='moveit_py_waferflow')
          self._arm = self._moveit.get_planning_component(self.PLANNING_GROUP)
        """
        self._node.get_logger().info('Initializing MoveItPy...')
        # TODO: uncomment when moveit_py is confirmed working in environment
        # self._moveit = MoveItPy(node_name='moveit_py_waferflow')
        # self._arm = self._moveit.get_planning_component(self.PLANNING_GROUP)
        self._node.get_logger().info('MoveItPy initialized (stub).')

    def plan_to_named_target(self, target_name: str) -> Optional[object]:
        """
        Plan to a named state from the SRDF (e.g. 'home', 'approach_cassette').

        Args:
            target_name: SRDF named state

        Returns:
            RobotTrajectory (or None on failure)

        TODO (Phase 6):
            self._arm.set_start_state_to_current_state()
            self._arm.set_goal_state(configuration_name=target_name)
            plan_result = self._arm.plan()
            if plan_result:
                return plan_result.trajectory
            return None
        """
        self._node.get_logger().info(f'[STUB] plan_to_named_target: {target_name}')
        return None  # TODO

    def plan_to_joint_target(self, joint_positions: dict) -> Optional[object]:
        """
        Plan to a specific joint configuration.
        """
        self._node.get_logger().info(f'plan_to_joint_target: {joint_positions}')
        
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        
        msg = JointTrajectory()
        msg.joint_names = list(joint_positions.keys())
        msg.header.stamp = self._node.get_clock().now().to_msg()
        
        pt = JointTrajectoryPoint()
        pt.positions = list(joint_positions.values())
        pt.time_from_start = Duration(sec=1, nanosec=0)
        
        msg.points.append(pt)
        return msg

    def plan_to_pose_target(self, target_pose,
                             link_name: str = None) -> Optional[object]:
        """
        Plan to a Cartesian pose target.
        """
        self._node.get_logger().info('[STUB] plan_to_pose_target - Not implemented for simple joints')
        return None

    def execute_trajectory(self, trajectory) -> bool:
        """
        Execute a JointTrajectory via publishing to /scara_arm_controller/joint_trajectory.
        """
        if trajectory is None:
            return False
            
        self._node.get_logger().info('execute_trajectory: publishing to controller')
        
        # We need a publisher in MoveItInterface or we can just use the FSM's.
        # FSM already has self._trajectory_pub. Let's just create one here.
        if not hasattr(self, '_traj_pub'):
            from trajectory_msgs.msg import JointTrajectory
            self._traj_pub = self._node.create_publisher(JointTrajectory, '/scara_arm_controller/joint_trajectory', 10)
            
        # Update timestamp
        trajectory.header.stamp = self._node.get_clock().now().to_msg()
        self._traj_pub.publish(trajectory)
        
        import time
        # Very simple wait logic
        if len(trajectory.points) > 0:
            duration = trajectory.points[-1].time_from_start.sec + trajectory.points[-1].time_from_start.nanosec * 1e-9
            time.sleep(duration + 0.1)
            
        return True
