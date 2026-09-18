#!/usr/bin/env python3
"""
jerk_limited_planner.py
========================
Phase 5: Wraps MoveIt-planned geometric paths with the S-curve time parameterization.

The flow:
  1. wafer_control requests a MoveIt plan (geometric path — joint waypoints, no timing).
  2. This module takes the raw waypoints and applies SCurveProfileND to retime them
     with the specified cycle_time and joint jerk limits.
  3. Returns a trajectory_msgs/JointTrajectory ready to send to the controller.

This replaces MoveIt's default TimeOptimalTrajectoryGeneration (TOTG) which uses
trapezoidal profiles. The S-curve retiming is the key innovation:
  - Same geometric path as MoveIt planned
  - Different timing: jerk-limited instead of jerk-unlimited
  - Configurable cycle time: faster = higher jerk, slower = lower jerk = less vibration

Throughput vs. precision/particle-risk tradeoff:
  Shorter cycle_time → higher jerk → more vibration → more particles on wafer surface
  Longer cycle_time → lower jerk  → less vibration → better placement accuracy
  The benchmark sweep quantifies this tradeoff across cycle_time ∈ [0.5, 2.0]s.
"""

import numpy as np
from typing import Optional
from .scurve_profile import SCurveProfileND

# ROS2 message types — imported here for use in to_joint_trajectory()
try:
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    ROS2_AVAILABLE = True
except ImportError:
    # Allow module to be used standalone (unit tests)
    ROS2_AVAILABLE = False

# Default joint limits (overridden from joint_limits.yaml at runtime)
DEFAULT_V_MAX  = [3.0, 3.0, 6.0, 0.3]    # [rad/s, rad/s, rad/s, m/s]
DEFAULT_A_MAX  = [6.0, 6.0, 12.0, 0.6]   # [rad/s², ...]
DEFAULT_J_MAX  = [30.0, 30.0, 60.0, 3.0] # [rad/s³, ...]


class JerkLimitedPlanner:
    """
    Retimes a geometric path (list of joint waypoints from MoveIt)
    using S-curve time parameterization.

    Usage:
        planner = JerkLimitedPlanner(joint_names, v_maxes, a_maxes, j_maxes)
        result = planner.retime(waypoints, cycle_time=1.0)
        ros_traj = result.to_joint_trajectory()
    """

    def __init__(self,
                 joint_names: list[str] = None,
                 v_maxes: list[float] = None,
                 a_maxes: list[float] = None,
                 j_maxes: list[float] = None,
                 velocity_scale: float = 1.0,
                 acceleration_scale: float = 1.0,
                 jerk_scale: float = 1.0):
        """
        Args:
            joint_names:  ordered list of joint names (matches URDF/MoveIt group)
            v_maxes:      per-joint max velocity limits [n_joints]
            a_maxes:      per-joint max acceleration limits [n_joints]
            j_maxes:      per-joint max jerk limits [n_joints]
            velocity_scale:     0.0–1.0 scaling factor applied to v_maxes
            acceleration_scale: 0.0–1.0 scaling factor applied to a_maxes
            jerk_scale:         0.0–1.0 scaling factor applied to j_maxes
        """
        self.joint_names = joint_names or ['joint_1', 'joint_2', 'joint_3', 'joint_z']
        n = len(self.joint_names)

        v_maxes = v_maxes or DEFAULT_V_MAX[:n]
        a_maxes = a_maxes or DEFAULT_A_MAX[:n]
        j_maxes = j_maxes or DEFAULT_J_MAX[:n]

        # Apply scaling factors
        v_scaled = [v * velocity_scale for v in v_maxes]
        a_scaled = [a * acceleration_scale for a in a_maxes]
        j_scaled = [j * jerk_scale for j in j_maxes]

        self._nd_planner = SCurveProfileND(
            joint_names=self.joint_names,
            v_maxes=v_scaled,
            a_maxes=a_scaled,
            j_maxes=j_scaled
        )
        self._dt = 0.01  # default sampling step [s]

    def retime(self, waypoints: np.ndarray,
                target_cycle_time: float,
                dt: float = 0.01) -> 'RetimingResult':
        """
        Apply S-curve time parameterization to a geometric path.

        Args:
            waypoints: [N × n_joints] array of joint positions from MoveIt
            target_cycle_time: desired total execution time [s]
            dt: sampling time step for the output trajectory [s]

        Returns:
            RetimingResult with time-stamped joint trajectory data.

        TODO (Phase 5):
          After implementing SCurveProfileND.plan_path() fully, this method
          should be straightforward. Test with a simple 2-waypoint path first.
        """
        if len(waypoints) < 2:
            raise ValueError('Need at least 2 waypoints.')

        waypoints_arr = np.array(waypoints, dtype=float)
        t_vec, q_traj, actual_time, peak_jerk = self._nd_planner.plan_path(
            waypoints=waypoints_arr,
            target_total_time=target_cycle_time,
            dt=dt
        )

        feasible = actual_time <= target_cycle_time * 1.01  # 1% tolerance

        return RetimingResult(
            joint_names=self.joint_names,
            t_vec=t_vec,
            q_traj=q_traj,
            actual_cycle_time=actual_time,
            peak_jerk=peak_jerk,
            feasible=feasible,
        )

    def load_limits_from_yaml(self, yaml_path: str) -> None:
        """
        Load joint limits from joint_limits.yaml and rebuild the planner.

        TODO (Phase 5): Implement YAML parsing using PyYAML.
        Expected format: same as wafer_moveit_config/config/joint_limits.yaml
        """
        raise NotImplementedError(
            'TODO (Phase 5): parse joint_limits.yaml and call __init__ '
            'with updated limits.'
        )


class RetimingResult:
    """
    Result of JerkLimitedPlanner.retime().
    Holds numpy arrays and provides conversion to ROS2 trajectory message.
    """

    def __init__(self, joint_names: list[str], t_vec: np.ndarray,
                 q_traj: np.ndarray, actual_cycle_time: float,
                 peak_jerk: float, feasible: bool):
        self.joint_names = joint_names
        self.t_vec = t_vec          # [M] time steps [s]
        self.q_traj = q_traj        # [M × n_joints] positions
        self.actual_cycle_time = actual_cycle_time
        self.peak_jerk = peak_jerk
        self.feasible = feasible

        # Compute velocities and accelerations via finite differences
        dt = np.diff(t_vec, prepend=t_vec[0] - (t_vec[1] - t_vec[0]))
        dt = np.maximum(dt, 1e-9)
        self.qd_traj = np.gradient(q_traj, t_vec, axis=0)   # velocities
        self.qdd_traj = np.gradient(self.qd_traj, t_vec, axis=0)  # accelerations

    def to_joint_trajectory(self) -> 'JointTrajectory':
        """
        Convert numpy arrays to a trajectory_msgs/JointTrajectory message.

        TODO (Phase 5): Implement this. Skeleton:
          msg = JointTrajectory()
          msg.joint_names = self.joint_names
          for i, t in enumerate(self.t_vec):
              pt = JointTrajectoryPoint()
              pt.positions = self.q_traj[i].tolist()
              pt.velocities = self.qd_traj[i].tolist()
              pt.accelerations = self.qdd_traj[i].tolist()
              pt.time_from_start = Duration(sec=int(t), nanosec=int((t%1)*1e9))
              msg.points.append(pt)
          return msg
        """
        if not ROS2_AVAILABLE:
            raise RuntimeError(
                'ROS2 not available. Call this method in a ROS2 context.'
            )

        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        for i, t in enumerate(self.t_vec):
            pt = JointTrajectoryPoint()
            pt.positions = self.q_traj[i].tolist()
            pt.velocities = self.qd_traj[i].tolist()
            pt.accelerations = self.qdd_traj[i].tolist()
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            msg.points.append(pt)

        return msg
