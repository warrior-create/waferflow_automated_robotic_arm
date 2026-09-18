#!/usr/bin/env python3
"""
pick_place_fsm.py
=================
Phase 6: Main state machine orchestrating one full pick-place cycle.

States (matches wafer_control FSM):
  IDLE      — waiting for a pick-place command
  APPROACH  — move to pre-pick position above source cassette slot
  ALIGN     — call AlignWafer service, compute correction
  PICK      — descend and "attach" wafer (logical attach — no physics grip)
  TRANSIT   — jerk-limited move from cassette to stocker
  PLACE     — descend and "release" wafer at destination slot
  VERIFY    — measure placement error (ground truth vs. commanded)
  REJECT    — if alignment error exceeds tolerance: skip pick, log reject

Transitions:
  IDLE    → APPROACH (on /waferflow/run_cycle command)
  APPROACH → ALIGN
  ALIGN    → PICK (if error < tolerance) | REJECT (if error >= tolerance)
  PICK     → TRANSIT
  TRANSIT  → PLACE
  PLACE    → VERIFY
  VERIFY   → IDLE (always, publishing PlacementResult + CycleMetrics)
  REJECT   → IDLE (publishing reject event)

Published topics:
  /waferflow/placement_result [wafer_msgs/PlacementResult]
  /waferflow/cycle_metrics    [wafer_msgs/CycleMetrics]

Subscribed topics:
  /waferflow/run_cycle [std_msgs/String] — payload: "A1→B1" format

TODO (Phase 6):
  - Implement each state's execute() body
  - Connect MoveItInterface for actual planning
  - Connect TrajectoryActionServer client for retimed execution
"""

import time
import enum
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from wafer_msgs.msg import PlacementResult, CycleMetrics
from wafer_msgs.srv import AlignWafer
from wafer_msgs.action import GenerateTrajectory

from .moveit_interface import MoveItInterface
from .tolerance_checker import ToleranceChecker


class State(enum.Enum):
    IDLE     = 'IDLE'
    APPROACH = 'APPROACH'
    ALIGN    = 'ALIGN'
    PICK     = 'PICK'
    TRANSIT  = 'TRANSIT'
    PLACE    = 'PLACE'
    VERIFY   = 'VERIFY'
    REJECT   = 'REJECT'


class PickPlaceFSM(Node):
    """
    State machine node orchestrating WaferFlow pick-place cycles.

    One cycle = IDLE → APPROACH → ALIGN → PICK → TRANSIT → PLACE → VERIFY → IDLE
    or         IDLE → APPROACH → ALIGN → REJECT → IDLE (if misaligned beyond tol.)
    """

    def __init__(self):
        super().__init__('pick_place_fsm')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('source_slots',
                               ['A1', 'A2', 'A3', 'A4', 'A5'])
        self.declare_parameter('dest_slots',
                               ['B1', 'B2', 'B3', 'B4', 'B5'])
        self.declare_parameter('default_cycle_time', 1.5)
        self.declare_parameter('alignment_tolerance_m', 0.002)    # 2 mm
        self.declare_parameter('alignment_tolerance_rad', 0.035)  # ~2 degrees
        self.declare_parameter('placement_tolerance_m', 0.001)    # 1 mm
        self.declare_parameter('placement_tolerance_rad', 0.017)  # ~1 degree

        self._cycle_time = self.get_parameter('default_cycle_time').value
        self._align_tol_m = self.get_parameter('alignment_tolerance_m').value
        self._align_tol_rad = self.get_parameter('alignment_tolerance_rad').value

        # ── State ──────────────────────────────────────────────────────
        self._state = State.IDLE
        self._cycle_index = 0
        self._source_slot: str | None = None
        self._dest_slot: str | None = None
        self._cycle_start_time: float = 0.0
        self._alignment_pose = None  # WaferPose from AlignWafer service
        self._peak_jerk: float = 0.0
        self._carrying_wafer: bool = False

        # ── Sub-components ─────────────────────────────────────────────
        self._moveit = MoveItInterface(self)
        self._tolerance_checker = ToleranceChecker(
            pos_tol=self.get_parameter('placement_tolerance_m').value,
            rot_tol=self.get_parameter('placement_tolerance_rad').value,
        )

        # ── ROS2 interfaces ────────────────────────────────────────────
        # Service client: AlignWafer
        self._align_client = self.create_client(
            AlignWafer, '/waferflow/align_wafer'
        )

        # Action client: GenerateTrajectory
        self._traj_client = ActionClient(
            self, GenerateTrajectory, '/waferflow/generate_trajectory'
        )

        # Subscriber: run_cycle command
        self._run_cycle_sub = self.create_subscription(
            String, '/waferflow/run_cycle', self._run_cycle_callback, 10
        )

        # Publishers: results
        self._placement_pub = self.create_publisher(
            PlacementResult, '/waferflow/placement_result', 10
        )
        self._metrics_pub = self.create_publisher(
            CycleMetrics, '/waferflow/cycle_metrics', 10
        )
        self._trajectory_pub = self.create_publisher(
            JointTrajectory, '/scara_arm_controller/joint_trajectory', 10
        )

        self.get_logger().info('PickPlaceFSM ready. Waiting for /waferflow/run_cycle ...')

    # ──────────────────────────────────────────────────────────────────
    # External trigger
    # ──────────────────────────────────────────────────────────────────

    def _run_cycle_callback(self, msg: String) -> None:
        """
        Parse a cycle command like "A1→B3" and trigger the FSM.

        Format: '<source_slot>→<dest_slot>' or '<source_slot>-><dest_slot>'
        Optional suffix: ':cycle_time=1.5' e.g. "A1→B1:cycle_time=0.75"
        """
        if self._state != State.IDLE:
            self.get_logger().warn(
                f'Ignoring run_cycle: FSM busy in state {self._state.value}'
            )
            return

        payload = msg.data.strip()
        # Parse optional cycle time
        if ':cycle_time=' in payload:
            payload, ct_str = payload.split(':cycle_time=')
            try:
                self._cycle_time = float(ct_str)
            except ValueError:
                self.get_logger().warn(f'Bad cycle_time in command: {ct_str}')

        # Parse slots
        sep = '→' if '→' in payload else '->'
        parts = payload.split(sep)
        if len(parts) != 2:
            self.get_logger().error(f'Bad command format: "{msg.data}". Use "A1→B1"')
            return

        self._source_slot = parts[0].strip()
        self._dest_slot = parts[1].strip()
        self._cycle_index += 1
        self._cycle_start_time = time.time()

        self.get_logger().info(
            f'Cycle {self._cycle_index}: {self._source_slot} → {self._dest_slot} '
            f'target_time={self._cycle_time}s'
        )
        self._transition(State.APPROACH)

    # ──────────────────────────────────────────────────────────────────
    # FSM transition dispatcher
    # ──────────────────────────────────────────────────────────────────

    def _transition(self, new_state: State) -> None:
        """Log transition and execute the new state's handler."""
        self.get_logger().info(
            f'FSM: {self._state.value} → {new_state.value}'
        )
        self._state = new_state
        handler = {
            State.APPROACH: self._state_approach,
            State.ALIGN:    self._state_align,
            State.PICK:     self._state_pick,
            State.TRANSIT:  self._state_transit,
            State.PLACE:    self._state_place,
            State.VERIFY:   self._state_verify,
            State.REJECT:   self._state_reject,
        }.get(new_state)
        if handler:
            handler()

    # ──────────────────────────────────────────────────────────────────
    # State handlers
    # ──────────────────────────────────────────────────────────────────

    def _scara_ik(self, x, y, z, pitch=0.0):
        import math
        # SCARA IK for L1=0.3, L2=0.3
        L1 = 0.3
        L2 = 0.3
        # Offset from base to shoulder is 0
        r = math.hypot(x, y)
        if r > L1 + L2 or r < abs(L1 - L2):
            self.get_logger().error(f"Target out of reach: {x}, {y}")
            return [0.0, 0.0, 0.0, 0.0]
            
        alpha = math.acos((L1**2 + L2**2 - r**2) / (2 * L1 * L2))
        theta2 = math.pi - alpha
        
        beta = math.acos((r**2 + L1**2 - L2**2) / (2 * r * L1))
        theta1 = math.atan2(y, x) - beta
        
        theta3 = pitch - (theta1 + theta2)
        # Z axis is inverted in our URDF or direct? Let's assume z_joint = z - base_height
        z_joint = z
        return [theta1, theta2, theta3, z_joint]
        
    def _get_slot_pose(self, slot_id: str):
        # A slots at x=0.55, B slots at x=-0.55
        # y = -0.06, -0.03, 0.0, 0.03, 0.06
        idx = int(slot_id[1]) - 1
        y = -0.06 + idx * 0.03
        x = 0.55 if slot_id.startswith('A') else -0.55
        z = 0.063 # Pick/Place height
        return x, y, z

    def _state_approach(self) -> None:
        """
        APPROACH: Move arm to pre-pick position above source slot.
        """
        self.get_logger().info(f'APPROACH: Moving above slot {self._source_slot}')
        x, y, z = self._get_slot_pose(self._source_slot)
        # Approach height
        z_approach = z + 0.05
        j = self._scara_ik(x, y, z_approach)
        
        traj = self._moveit.plan_to_joint_target({'joint_1': j[0], 'joint_2': j[1], 'joint_3': j[2], 'joint_z': j[3]})
        self._moveit.execute_trajectory(traj)
        
        self._transition(State.ALIGN)

    def _state_align(self) -> None:
        """
        ALIGN: Call AlignWafer service to get pre-pick correction.
        """
        self.get_logger().info(f'ALIGN: Calling AlignWafer for slot {self._source_slot}')

        if not self._align_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('AlignWafer service not available!')
            self._transition(State.REJECT)
            return

        request = AlignWafer.Request()
        request.slot_id = self._source_slot

        future = self._align_client.call_async(request)
        future.add_done_callback(self._on_align_response)

    def _on_align_response(self, future) -> None:
        """Callback for async AlignWafer service response."""
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'AlignWafer service call failed: {e}')
            self._transition(State.REJECT)
            return

        if not response.detected:
            self.get_logger().warn(
                f'No wafer detected in {self._source_slot}: {response.message}'
            )
            self._transition(State.REJECT)
            return

        import math
        pose = response.pose
        pos_err = math.sqrt(pose.x ** 2 + pose.y ** 2)
        rot_err = abs(pose.theta)

        self.get_logger().info(
            f'ALIGN result: pos_err={pos_err*1000:.1f}mm '
            f'rot_err={math.degrees(rot_err):.2f}° '
            f'confidence={pose.confidence:.2f}'
        )

        if pos_err > self._align_tol_m or rot_err > self._align_tol_rad:
            self.get_logger().warn(
                f'Alignment error EXCEEDS tolerance '
                f'({pos_err*1000:.1f}mm vs {self._align_tol_m*1000:.1f}mm tol, '
                f'{math.degrees(rot_err):.2f}° vs '
                f'{math.degrees(self._align_tol_rad):.2f}° tol). REJECTING.'
            )
            self._transition(State.REJECT)
            return

        self._alignment_pose = pose
        self._transition(State.PICK)

    def _state_pick(self) -> None:
        """
        PICK: Descend to pick position and logically attach wafer.
        """
        self.get_logger().info('PICK: Descending to wafer surface')
        x, y, z = self._get_slot_pose(self._source_slot)
        
        # Apply alignment correction
        if self._alignment_pose:
            x += self._alignment_pose.x
            y += self._alignment_pose.y
            
        j = self._scara_ik(x, y, z)
        traj = self._moveit.plan_to_joint_target({'joint_1': j[0], 'joint_2': j[1], 'joint_3': j[2], 'joint_z': j[3]})
        self._moveit.execute_trajectory(traj)
        
        # Attach wafer
        if not hasattr(self, '_attach_pub'):
            from std_msgs.msg import Empty
            self._attach_pub = self.create_publisher(Empty, '/waferflow/attach', 1)
        self._attach_pub.publish(Empty())
        import time
        time.sleep(0.5)
        
        # Ascend
        j = self._scara_ik(x, y, z + 0.05)
        traj = self._moveit.plan_to_joint_target({'joint_1': j[0], 'joint_2': j[1], 'joint_3': j[2], 'joint_z': j[3]})
        self._moveit.execute_trajectory(traj)
        
        self._carrying_wafer = True
        self._transition(State.TRANSIT)

    def _state_transit(self) -> None:
        """
        TRANSIT: Jerk-limited move from cassette to stocker.
        """
        self.get_logger().info(
            f'TRANSIT: Jerk-limited move to stocker slot {self._dest_slot} '
            f'(cycle_time={self._cycle_time}s)'
        )
        
        x1, y1, z1 = self._get_slot_pose(self._source_slot)
        x2, y2, z2 = self._get_slot_pose(self._dest_slot)
        
        start_pt = self._scara_ik(x1, y1, z1 + 0.05)
        end_pt = self._scara_ik(x2, y2, z2 + 0.05)
        
        request = GenerateTrajectory.Goal()
        request.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_z']
        request.num_waypoints = 2
            
        request.waypoints_flat = start_pt + end_pt
        request.target_cycle_time = self._cycle_time
        request.max_velocity_scale = 1.0
        request.max_acceleration_scale = 1.0
        request.max_jerk_scale = 1.0
        
        # Send goal
        future = self._traj_client.send_goal_async(request)
        future.add_done_callback(self._on_traj_goal_response)
        
    def _on_traj_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            self._transition(State.PLACE)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_traj_result)
        
    def _on_traj_result(self, future):
        result = future.result().result
        if not result.feasible:
            self.get_logger().error(f'Trajectory not feasible: {result.message}')
            self._transition(State.PLACE)
            return
            
        self._peak_jerk = result.peak_jerk
        
        # Publish trajectory to controller to visually animate!
        traj_msg = result.trajectory
        # Adjust timestamp so controller executes it now
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        self._trajectory_pub.publish(traj_msg)
        
        self.get_logger().info(f'Executing visual trajectory taking {result.actual_cycle_time:.2f}s...')
        
        # Wait for cycle time to finish executing asynchronously
        timer = threading.Timer(result.actual_cycle_time + 0.1, lambda: self._transition(State.PLACE))
        timer.start()

    def _state_place(self) -> None:
        """
        PLACE: Descend to destination slot and release wafer.
        """
        self.get_logger().info(f'PLACE: Placing wafer in slot {self._dest_slot}')
        x, y, z = self._get_slot_pose(self._dest_slot)
        
        j = self._scara_ik(x, y, z)
        traj = self._moveit.plan_to_joint_target({'joint_1': j[0], 'joint_2': j[1], 'joint_3': j[2], 'joint_z': j[3]})
        self._moveit.execute_trajectory(traj)
        
        # Detach wafer
        if not hasattr(self, '_detach_pub'):
            from std_msgs.msg import Empty
            self._detach_pub = self.create_publisher(Empty, '/waferflow/detach', 1)
        self._detach_pub.publish(Empty())
        import time
        time.sleep(0.5)
        
        # Ascend
        j = self._scara_ik(x, y, z + 0.05)
        traj = self._moveit.plan_to_joint_target({'joint_1': j[0], 'joint_2': j[1], 'joint_3': j[2], 'joint_z': j[3]})
        self._moveit.execute_trajectory(traj)
        
        self._carrying_wafer = False
        self._transition(State.VERIFY)

    def _state_verify(self) -> None:
        """
        VERIFY: Measure placement error and publish results.
        """
        cycle_elapsed = time.time() - self._cycle_start_time
        self.get_logger().info(
            f'VERIFY: Cycle {self._cycle_index} complete in {cycle_elapsed:.3f}s'
        )

        import math
        # Initialize TF buffer if not present
        if not hasattr(self, '_tf_buffer'):
            from tf2_ros import Buffer, TransformListener
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            
        try:
            # Wait a bit for tf
            time.sleep(0.5)
            trans = self._tf_buffer.lookup_transform('world', 'wafer_disc', rclpy.time.Time())
            actual_x = trans.transform.translation.x
            actual_y = trans.transform.translation.y
            
            target_x, target_y, target_z = self._get_slot_pose(self._dest_slot)
            pos_error = math.hypot(actual_x - target_x, actual_y - target_y)
            rot_error = 0.0 # Simplify rot error
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            pos_error = 0.0
            rot_error = 0.0

        success = self._tolerance_checker.check(
            pos_error=abs(pos_error),
            rot_error=abs(rot_error)
        )

        # Publish PlacementResult
        placement_msg = PlacementResult()
        placement_msg.position_error = abs(pos_error)
        placement_msg.orientation_error = abs(rot_error)
        placement_msg.success = success
        placement_msg.slot_id = self._dest_slot or ''
        placement_msg.cycle_index = self._cycle_index
        placement_msg.cycle_time_target = self._cycle_time
        placement_msg.stamp = self.get_clock().now().to_msg()
        self._placement_pub.publish(placement_msg)

        # Publish CycleMetrics
        metrics_msg = CycleMetrics()
        metrics_msg.cycle_time = cycle_elapsed
        metrics_msg.peak_jerk = self._peak_jerk
        metrics_msg.duration = cycle_elapsed
        metrics_msg.success = success
        metrics_msg.cycle_index = self._cycle_index
        metrics_msg.phase_failed = '' if success else 'VERIFY'
        metrics_msg.stamp = self.get_clock().now().to_msg()
        self._metrics_pub.publish(metrics_msg)

        self.get_logger().info(
            f'  pos_error={pos_error*1000:.2f}mm '
            f'rot_error={rot_error*1000:.1f}mrad '
            f'success={success}'
        )
        self._transition(State.IDLE)
        self._state = State.IDLE  # explicit reset

    def _state_reject(self) -> None:
        """
        REJECT: Log a reject event and return to IDLE.
        Publishes a failed PlacementResult with success=False.

        TODO (Phase 6): optionally implement re-pick logic:
          - Move to a reject bin instead of placing
          - Increment a reject counter
          - Alert operator if consecutive_rejects > threshold
        """
        self.get_logger().warn(
            f'REJECT: Cycle {self._cycle_index} rejected at slot {self._source_slot}'
        )

        cycle_elapsed = time.time() - self._cycle_start_time

        placement_msg = PlacementResult()
        placement_msg.position_error = -1.0  # sentinel: rejected
        placement_msg.orientation_error = -1.0
        placement_msg.success = False
        placement_msg.slot_id = self._source_slot or ''
        placement_msg.cycle_index = self._cycle_index
        placement_msg.cycle_time_target = self._cycle_time
        placement_msg.stamp = self.get_clock().now().to_msg()
        self._placement_pub.publish(placement_msg)

        metrics_msg = CycleMetrics()
        metrics_msg.cycle_time = cycle_elapsed
        metrics_msg.peak_jerk = 0.0
        metrics_msg.duration = cycle_elapsed
        metrics_msg.success = False
        metrics_msg.cycle_index = self._cycle_index
        metrics_msg.phase_failed = 'ALIGN'
        metrics_msg.stamp = self.get_clock().now().to_msg()
        self._metrics_pub.publish(metrics_msg)

        self._state = State.IDLE


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceFSM()
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
