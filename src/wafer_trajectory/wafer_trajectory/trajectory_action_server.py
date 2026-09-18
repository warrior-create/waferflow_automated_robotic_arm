#!/usr/bin/env python3
"""
trajectory_action_server.py
============================
Phase 5: Exposes the GenerateTrajectory action server.

Action: /waferflow/generate_trajectory  [wafer_msgs/action/GenerateTrajectory]
  Goal:     joint waypoints (flat array) + target cycle time + scale factors
  Feedback: progress % and current phase
  Result:   JointTrajectory + actual_cycle_time + peak_jerk + feasibility flag

wafer_control calls this action server to obtain the jerk-limited retimed
trajectory before handing it to the JointTrajectoryController for execution.

The separation of planning (this action server) from execution (ros2_control)
allows the benchmark to measure planning latency separately from execution time.

TODO (Phase 5):
  - Connect to JerkLimitedPlanner once scurve_profile.py is fully implemented
  - Add real-time feedback publishing (progress %)
  - Handle preemption (goal cancellation mid-planning)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from wafer_msgs.action import GenerateTrajectory
from .jerk_limited_planner import JerkLimitedPlanner


class TrajectoryActionServer(Node):
    """
    ROS2 Action Server exposing the GenerateTrajectory action.

    Accepts a geometric path (joint waypoints) and a target cycle time,
    applies S-curve retiming, and returns a JointTrajectory.
    """

    def __init__(self):
        super().__init__('trajectory_action_server')

        # ── Parameters (loaded from system_params.yaml via launch) ─────
        self.declare_parameter('default_velocity_scale', 1.0)
        self.declare_parameter('default_acceleration_scale', 1.0)
        self.declare_parameter('default_jerk_scale', 1.0)
        self.declare_parameter('sampling_dt', 0.01)
        self.declare_parameter('joint_names',
                               ['joint_1', 'joint_2', 'joint_3', 'joint_z'])

        self._v_scale  = self.get_parameter('default_velocity_scale').value
        self._a_scale  = self.get_parameter('default_acceleration_scale').value
        self._j_scale  = self.get_parameter('default_jerk_scale').value
        self._dt       = self.get_parameter('sampling_dt').value
        self._joint_names = self.get_parameter('joint_names').value

        # ── Planner ────────────────────────────────────────────────────
        self._planner = JerkLimitedPlanner(
            joint_names=self._joint_names,
            velocity_scale=self._v_scale,
            acceleration_scale=self._a_scale,
            jerk_scale=self._j_scale,
        )

        # ── Action Server ──────────────────────────────────────────────
        cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            GenerateTrajectory,
            '/waferflow/generate_trajectory',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group
        )

        self.get_logger().info('TrajectoryActionServer ready at /waferflow/generate_trajectory')

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Validate incoming goal — reject if waypoints are malformed."""
        n_joints = len(goal_request.joint_names)
        n_pts = goal_request.num_waypoints
        expected_flat_len = n_joints * n_pts

        if len(goal_request.waypoints_flat) != expected_flat_len:
            self.get_logger().error(
                f'Goal rejected: waypoints_flat has {len(goal_request.waypoints_flat)} '
                f'elements but expected {expected_flat_len} '
                f'({n_pts} pts × {n_joints} joints)'
            )
            return GoalResponse.REJECT

        if goal_request.target_cycle_time <= 0:
            self.get_logger().error('Goal rejected: target_cycle_time must be > 0')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        """Accept cancellation — planning is fast enough that this is low-risk."""
        self.get_logger().info('TrajectoryActionServer: cancel requested.')
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle) -> GenerateTrajectory.Result:
        """
        Main execution: retime the geometric path with S-curve profile.

        Steps:
          1. Unpack waypoints from flat array to [N × n_joints]
          2. Apply per-goal scale factors to planner limits
          3. Call JerkLimitedPlanner.retime()
          4. Publish feedback at key phases
          5. Convert result to JointTrajectory and return

        TODO (Phase 5):
          - Publish real-time feedback during planning (currently one-shot)
          - Handle goal cancellation mid-planning
        """
        goal = goal_handle.request
        self.get_logger().info(
            f'Executing trajectory generation: '
            f'{goal.num_waypoints} waypoints, '
            f'target_time={goal.target_cycle_time:.2f}s'
        )

        # ── Publish initial feedback ───────────────────────────────────
        feedback = GenerateTrajectory.Feedback()
        feedback.progress_percent = 0.0
        feedback.current_phase = 'UNPACKING_WAYPOINTS'
        goal_handle.publish_feedback(feedback)

        # ── Unpack waypoints ───────────────────────────────────────────
        n_joints = len(goal.joint_names)
        n_pts = goal.num_waypoints
        waypoints_flat = list(goal.waypoints_flat)
        waypoints = np.array(waypoints_flat).reshape(n_pts, n_joints)

        # ── Rebuild planner with goal-specific scale factors ───────────
        planner = JerkLimitedPlanner(
            joint_names=list(goal.joint_names),
            velocity_scale=goal.max_velocity_scale,
            acceleration_scale=goal.max_acceleration_scale,
            jerk_scale=goal.max_jerk_scale,
        )

        feedback.progress_percent = 20.0
        feedback.current_phase = 'COMPUTING_SCURVE'
        goal_handle.publish_feedback(feedback)

        # ── Run S-curve retiming ───────────────────────────────────────
        try:
            result_data = planner.retime(
                waypoints=waypoints,
                target_cycle_time=goal.target_cycle_time,
                dt=self._dt
            )
        except Exception as e:
            self.get_logger().error(f'Retiming failed: {e}')
            goal_handle.abort()
            result = GenerateTrajectory.Result()
            result.feasible = False
            result.message = str(e)
            return result

        feedback.progress_percent = 80.0
        feedback.current_phase = 'BUILDING_ROS_TRAJECTORY'
        goal_handle.publish_feedback(feedback)

        # ── Build ROS2 JointTrajectory message ─────────────────────────
        try:
            trajectory_msg = result_data.to_joint_trajectory()
        except Exception as e:
            self.get_logger().error(f'Trajectory message conversion failed: {e}')
            goal_handle.abort()
            result = GenerateTrajectory.Result()
            result.feasible = False
            result.message = str(e)
            return result

        feedback.progress_percent = 100.0
        feedback.current_phase = 'DONE'
        goal_handle.publish_feedback(feedback)

        # ── Return result ──────────────────────────────────────────────
        goal_handle.succeed()

        result = GenerateTrajectory.Result()
        result.trajectory = trajectory_msg
        result.actual_cycle_time = result_data.actual_cycle_time
        result.peak_jerk = result_data.peak_jerk
        result.feasible = result_data.feasible
        result.message = ('OK' if result_data.feasible
                          else f'Minimum time exceeded target by '
                               f'{result_data.actual_cycle_time - goal.target_cycle_time:.3f}s')

        self.get_logger().info(
            f'Trajectory generated: actual_time={result.actual_cycle_time:.3f}s '
            f'peak_jerk={result.peak_jerk:.2f} rad/s³ '
            f'feasible={result.feasible}'
        )
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
