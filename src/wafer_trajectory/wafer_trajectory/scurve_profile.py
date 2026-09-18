#!/usr/bin/env python3
"""
scurve_profile.py
=================
THE KEY DIFFERENTIATOR — Phase 5.

Standalone S-curve (jerk-limited) velocity profile generator.
This module has ZERO ROS2 dependencies so it can be unit-tested independently
with matplotlib before any ROS integration.

Theory:
  A trapezoidal velocity profile has discontinuous acceleration (jerk = ∞ at
  ramp transitions). High jerk causes mechanical vibration and, in a semiconductor
  fab, generates particles that contaminate wafers.

  An S-curve profile limits jerk to a maximum value J_max by replacing the
  instantaneous ramp transitions with smooth cubic segments. The result is a
  velocity profile shaped like a stretched "S" during acceleration and deceleration.

Seven-phase S-curve:
  Phase 1: Jerk up    (constant +J_max) — acceleration increases
  Phase 2: Const accel (zero jerk)      — optional if A_max reachable
  Phase 3: Jerk down  (constant -J_max) — acceleration decreases
  Phase 4: Constant velocity
  Phase 5: Jerk down  (constant -J_max) — deceleration increases
  Phase 6: Const decel (zero jerk)      — optional
  Phase 7: Jerk up    (constant +J_max) — deceleration decreases

Key constraint: given a target total cycle time T, the profile is scaled to
fit exactly within T while respecting V_max, A_max, J_max.
If T is too short to respect all limits, the profile is clamped and
`feasible=False` is returned.

Usage (standalone, no ROS):
    from scurve_profile import SCurveProfile1D, SCurveProfileND

    prof = SCurveProfile1D(v_max=3.0, a_max=6.0, j_max=30.0)
    result = prof.plan(q0=0.0, q1=1.5, target_time=0.8)
    t, pos, vel, acc, jrk = result.sample(dt=0.001)

References:
  - Biagiotti, L., Melchiorri, C. (2008). Trajectory Planning for Automatic
    Machines and Robots. Springer. Chapter 3.
  - Kyriakopoulos, K.J., Saridis, G.N. (1988). Minimum jerk path generation.
    IEEE ICRA.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional
import warnings


@dataclass
class SCurveResult1D:
    """
    Result of a 1D S-curve planning call.
    Stores the 7 phase durations and motion parameters.
    """
    q0: float          # start position
    q1: float          # end position
    v_max: float       # peak velocity achieved (may be < v_limit)
    a_max: float       # peak acceleration achieved
    j_max: float       # jerk limit used
    t_phases: np.ndarray   # [t1, t2, t3, t4, t5, t6, t7] phase durations [s]
    total_time: float      # sum of t_phases
    feasible: bool         # True if all limits respected; False if clamped
    peak_jerk: float       # maximum |jerk| = j_max (by definition for S-curve)

    def sample(self, dt: float = 0.001) -> tuple:
        """
        Sample the trajectory at uniform time steps.

        Returns:
            t:   time vector [N]
            pos: position [N]
            vel: velocity [N]
            acc: acceleration [N]
            jrk: jerk [N]
        """
        t_vec = np.arange(0.0, self.total_time + dt, dt)
        pos = np.zeros_like(t_vec)
        vel = np.zeros_like(t_vec)
        acc = np.zeros_like(t_vec)
        jrk = np.zeros_like(t_vec)

        direction = np.sign(self.q1 - self.q0)
        T = self.t_phases

        for i, t in enumerate(t_vec):
            p, v, a, j = self._eval_at(t, direction, T)
            pos[i] = p
            vel[i] = v
            acc[i] = a
            jrk[i] = j

        return t_vec, pos, vel, acc, jrk

    def _eval_at(self, t: float, direction: float,
                  T: np.ndarray) -> tuple[float, float, float, float]:
        """
        Evaluate position/velocity/acceleration/jerk at time t for the 7 phases.
        """
        t1, t2, t3, t4, t5, t6, t7 = T
        J = self.j_max * direction

        t_clamp = min(t, self.total_time)
        t_clamp = max(t_clamp, 0.0)

        # Boundaries of the phases
        T1 = t1
        T2 = T1 + t2
        T3 = T2 + t3
        T4 = T3 + t4
        T5 = T4 + t5
        T6 = T5 + t6
        T7 = T6 + t7

        # Base conditions at start of each phase
        # Phase 1: t in [0, T1] (Jerk = J)
        v0 = 0.0
        a0 = 0.0
        p0 = self.q0
        
        # End of Phase 1
        v1 = v0 + a0 * t1 + 0.5 * J * t1**2
        a1 = a0 + J * t1
        p1 = p0 + v0 * t1 + 0.5 * a0 * t1**2 + (1.0/6.0) * J * t1**3

        # End of Phase 2: t in [T1, T2] (Jerk = 0)
        v2 = v1 + a1 * t2
        a2 = a1
        p2 = p1 + v1 * t2 + 0.5 * a1 * t2**2

        # End of Phase 3: t in [T2, T3] (Jerk = -J)
        v3 = v2 + a2 * t3 - 0.5 * J * t3**2
        a3 = a2 - J * t3
        p3 = p2 + v2 * t3 + 0.5 * a2 * t3**2 - (1.0/6.0) * J * t3**3

        # End of Phase 4: t in [T3, T4] (Jerk = 0, Acc = 0)
        v4 = v3
        a4 = 0.0
        p4 = p3 + v3 * t4

        # End of Phase 5: t in [T4, T5] (Jerk = -J)
        v5 = v4 + a4 * t5 - 0.5 * J * t5**2
        a5 = a4 - J * t5
        p5 = p4 + v4 * t5 + 0.5 * a4 * t5**2 - (1.0/6.0) * J * t5**3

        # End of Phase 6: t in [T5, T6] (Jerk = 0)
        v6 = v5 + a5 * t6
        a6 = a5
        p6 = p5 + v5 * t6 + 0.5 * a5 * t6**2

        # End of Phase 7 is q1
        
        # Evaluate based on phase
        if t_clamp <= T1:
            tau = t_clamp
            j = J
            a = a0 + j * tau
            v = v0 + a0 * tau + 0.5 * j * tau**2
            p = p0 + v0 * tau + 0.5 * a0 * tau**2 + (1.0/6.0) * j * tau**3
        elif t_clamp <= T2:
            tau = t_clamp - T1
            j = 0.0
            a = a1
            v = v1 + a1 * tau
            p = p1 + v1 * tau + 0.5 * a1 * tau**2
        elif t_clamp <= T3:
            tau = t_clamp - T2
            j = -J
            a = a2 + j * tau
            v = v2 + a2 * tau + 0.5 * j * tau**2
            p = p2 + v2 * tau + 0.5 * a2 * tau**2 + (1.0/6.0) * j * tau**3
        elif t_clamp <= T4:
            tau = t_clamp - T3
            j = 0.0
            a = 0.0
            v = v3
            p = p3 + v3 * tau
        elif t_clamp <= T5:
            tau = t_clamp - T4
            j = -J
            a = a4 + j * tau
            v = v4 + a4 * tau + 0.5 * j * tau**2
            p = p4 + v4 * tau + 0.5 * a4 * tau**2 + (1.0/6.0) * j * tau**3
        elif t_clamp <= T6:
            tau = t_clamp - T5
            j = 0.0
            a = a5
            v = v5 + a5 * tau
            p = p5 + v5 * tau + 0.5 * a5 * tau**2
        else:
            tau = min(t_clamp - T6, t7)
            j = J
            a = a6 + j * tau
            v = v6 + a6 * tau + 0.5 * j * tau**2
            p = p6 + v6 * tau + 0.5 * a6 * tau**2 + (1.0/6.0) * j * tau**3
            
        if t_clamp >= self.total_time:
            p = self.q1
            v = 0.0
            a = 0.0
            j = 0.0

        return p, v, a, j


class SCurveProfile1D:
    """
    1D S-curve (jerk-limited) trajectory planner.

    Plans a move from q0 to q1 in exactly target_time seconds (if feasible),
    respecting velocity, acceleration, and jerk limits.

    This is the core algorithm — all N-DOF planning in SCurveProfileND
    uses this as a building block per joint, then scales to a common time.
    """

    def __init__(self, v_max: float, a_max: float, j_max: float):
        """
        Args:
            v_max: maximum velocity [units/s]
            a_max: maximum acceleration [units/s²]
            j_max: maximum jerk [units/s³]
        """
        if v_max <= 0 or a_max <= 0 or j_max <= 0:
            raise ValueError('All limits must be positive.')
        self.v_max = v_max
        self.a_max = a_max
        self.j_max = j_max

    def plan(self, q0: float, q1: float,
              target_time: float) -> SCurveResult1D:
        """
        Plan a 1D move from q0 to q1 with a given target total time.

        The algorithm:
          1. Compute the minimum feasible time T_min respecting all limits.
          2. If target_time >= T_min: scale the profile to hit exactly target_time
             by extending the constant-velocity phase (Phase 4).
          3. If target_time < T_min: set feasible=False, use T_min (max speed).

        Args:
            q0: start position [units]
            q1: end position [units]
            target_time: desired total time [s]

        Returns:
            SCurveResult1D with all phase durations computed.

        TODO (Phase 5): Implement the full 7-phase planning equations.
          Key equations (from Biagiotti & Melchiorri Ch.3):
            t_j1 = A_max / J_max             — jerk phase duration
            t_a  = (V_max - 0) / A_max - t_j1 — const-accel duration
                   (= 0 if V_max reachable without full A_max)
            t_v  = (q1 - q0) / V_max - (t_j1 + t_a/2 + t_j1/2)
                   (= 0 if distance too short for full speed)
            Total T_min = 2*(t_j1 + t_a + t_j1) + t_v + 2*(t_j1 + t_a + t_j1)
                        = 2*t_a + 4*t_j1 + t_v  (simplified symmetric case)
        """
        distance = abs(q1 - q0)

        if distance < 1e-9:
            # Zero motion — return trivial result
            return SCurveResult1D(
                q0=q0, q1=q1, v_max=0.0, a_max=0.0, j_max=self.j_max,
                t_phases=np.zeros(7), total_time=max(target_time, 0.001),
                feasible=True, peak_jerk=0.0
            )

        # ── Step 1: compute jerk phase duration ─────────────────────
        t_j = self.a_max / self.j_max

        # ── Step 2: check if full A_max is reached ───────────────────
        v_peak_if_full_a = self.a_max * t_j  # v at end of jerk-up phase alone
        # If V_max reachable in time 2*t_j:
        if 2 * v_peak_if_full_a <= self.v_max:
            # Full A_max profile
            t_a = (self.v_max - 2 * v_peak_if_full_a) / self.a_max
            v_peak = self.v_max
        else:
            # Triangular accel — A_max not fully reached
            t_j = np.sqrt(self.v_max / self.j_max)
            t_a = 0.0
            v_peak = self.v_max

        # ── Step 3: minimum distance to reach V_max ──────────────────
        d_min = (t_j + t_a / 2.0) * v_peak * 2  # accel + decel, simplified

        # ── Step 4: adjust V_peak if distance too short ───────────────
        if d_min > distance:
            # Can't reach v_max — solve for reduced v_peak
            # v_peak = sqrt(distance * j_max) (triangular, t_a=0)
            v_peak = np.sqrt(distance * self.j_max)
            v_peak = min(v_peak, self.v_max)
            t_j = np.sqrt(v_peak / self.j_max)
            t_a = 0.0

        # ── Step 5: compute constant-velocity duration ────────────────
        t_v = (distance - d_min) / v_peak if d_min < distance else 0.0
        t_v = max(t_v, 0.0)

        # ── Step 6: assemble 7 phases ─────────────────────────────────
        # Symmetric profile: [t_j, t_a, t_j, t_v, t_j, t_a, t_j]
        T_phases = np.array([t_j, t_a, t_j, t_v, t_j, t_a, t_j])
        T_min = float(np.sum(T_phases))

        # ── Step 7: stretch to target_time via const-velocity ─────────
        feasible = target_time >= T_min
        if feasible and target_time > T_min:
            T_phases[3] += (target_time - T_min)  # extend Phase 4
        elif not feasible:
            warnings.warn(
                f'Target time {target_time:.3f}s < minimum {T_min:.3f}s. '
                f'Using minimum time. Peak velocity and jerk at limits.',
                UserWarning, stacklevel=2
            )

        total_time = float(np.sum(T_phases))

        return SCurveResult1D(
            q0=q0, q1=q1,
            v_max=v_peak, a_max=self.a_max,
            j_max=self.j_max,
            t_phases=T_phases,
            total_time=total_time,
            feasible=feasible,
            peak_jerk=self.j_max
        )

    def min_time(self, distance: float) -> float:
        """
        Compute the minimum feasible time to travel `distance`.
        Useful for computing per-joint minimum times in N-DOF planning.
        """
        result = self.plan(0.0, distance, target_time=0.0)
        return result.total_time


class SCurveProfileND:
    """
    N-dimensional S-curve trajectory planner.

    Extends SCurveProfile1D to handle multi-joint trajectories:
    For each segment (waypoint-to-waypoint), plan each joint independently
    with its own limits, then scale all joints to the same duration
    (the joint requiring the most time determines the segment duration).

    This is the time-parameterization algorithm that replaces MoveIt's
    default TimeOptimalTrajectoryGeneration (TOTG). The difference:
      - TOTG uses trapezoidal profiles (jerk unlimited at transitions)
      - SCurveProfileND uses 7-phase S-curves (jerk bounded by j_max per joint)
    """

    def __init__(self, joint_names: list[str],
                 v_maxes: list[float],
                 a_maxes: list[float],
                 j_maxes: list[float]):
        """
        Args:
            joint_names: ordered list of joint names
            v_maxes: max velocity per joint [units/s]
            a_maxes: max acceleration per joint [units/s²]
            j_maxes: max jerk per joint [units/s³]
        """
        assert len(joint_names) == len(v_maxes) == len(a_maxes) == len(j_maxes)
        self.joint_names = joint_names
        self.n_joints = len(joint_names)
        self._planners = [
            SCurveProfile1D(v, a, j)
            for v, a, j in zip(v_maxes, a_maxes, j_maxes)
        ]

    def plan_segment(self, q_start: np.ndarray, q_end: np.ndarray,
                      target_time: float) -> list[SCurveResult1D]:
        """
        Plan a single segment (one pair of waypoints) for all joints.

        Algorithm:
          1. For each joint, compute the minimum feasible time given its limits.
          2. The segment time = max(target_time, max(per-joint min times)).
          3. Re-plan each joint with the common segment time (stretches via Phase 4).

        Args:
            q_start: starting joint positions [n_joints]
            q_end: ending joint positions [n_joints]
            target_time: desired segment duration [s]

        Returns:
            list of SCurveResult1D, one per joint, all with same total_time.
        """
        assert len(q_start) == len(q_end) == self.n_joints

        # ── Pass 1: find bottleneck joint ─────────────────────────────
        min_times = [
            self._planners[i].min_time(abs(q_end[i] - q_start[i]))
            for i in range(self.n_joints)
        ]
        t_min_global = max(min_times)
        t_segment = max(target_time, t_min_global)

        # ── Pass 2: plan each joint with common duration ───────────────
        results = [
            self._planners[i].plan(
                q0=float(q_start[i]),
                q1=float(q_end[i]),
                target_time=t_segment
            )
            for i in range(self.n_joints)
        ]
        return results

    def plan_path(self, waypoints: np.ndarray,
                   target_total_time: float,
                   dt: float = 0.01) -> tuple[np.ndarray, np.ndarray, float, float]:
        """
        Plan a full multi-waypoint path using S-curve time parameterization.

        Args:
            waypoints: [N_waypoints × n_joints] array of joint positions
            target_total_time: desired total path duration [s]
            dt: sampling time step for output trajectory [s]

        Returns:
            t_vec:    time vector [M]
            q_traj:  joint positions [M × n_joints]
            total_time: actual total time achieved
            peak_jerk: max jerk norm across all joints and all time steps

        TODO (Phase 5): Implement proper per-segment time allocation.
          Current approach: divide target_total_time equally across segments.
          Better approach: allocate proportional to per-segment arc length.
        """
        N = len(waypoints)
        if N < 2:
            raise ValueError('Need at least 2 waypoints.')

        n_segments = N - 1
        t_per_segment = target_total_time / n_segments

        all_t = []
        all_q = []
        t_offset = 0.0
        peak_jerk = 0.0

        for seg in range(n_segments):
            q_start = waypoints[seg]
            q_end = waypoints[seg + 1]

            results = self.plan_segment(q_start, q_end, t_per_segment)
            seg_duration = results[0].total_time

            # Sample all joints at dt
            t_local = np.arange(0.0, seg_duration + dt, dt)
            q_seg = np.zeros((len(t_local), self.n_joints))

            for j, res in enumerate(results):
                _, pos_j, _, _, jrk_j = res.sample(dt=dt)
                n = min(len(t_local), len(pos_j))
                q_seg[:n, j] = pos_j[:n]
                peak_jerk = max(peak_jerk, float(np.max(np.abs(jrk_j))))

            # Avoid duplicate time points between segments
            if seg > 0:
                t_local = t_local[1:]
                q_seg = q_seg[1:]

            all_t.append(t_local + t_offset)
            all_q.append(q_seg)
            t_offset += seg_duration

        t_vec = np.concatenate(all_t)
        q_traj = np.concatenate(all_q, axis=0)
        total_time = float(t_vec[-1])

        return t_vec, q_traj, total_time, peak_jerk
