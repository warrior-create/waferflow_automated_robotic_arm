#!/usr/bin/env python3
"""
alignment_service.py
====================
Phase 4: Exposes the AlignWafer ROS2 service.

Service: /waferflow/align_wafer  [wafer_msgs/srv/AlignWafer]
  Request:  slot_id (string)
  Response: WaferPose (x, y, theta, confidence), detected (bool), message (string)

This node holds a reference to CameraSimNode's latest detections and
serves them via the ROS2 service interface. wafer_control's FSM calls
this before every PICK to obtain the pre-alignment correction.

Architecture note:
  The service is intentionally *synchronous* (returns cached data) rather
  than triggering a new capture, to keep latency predictable. The camera
  node runs at 30 Hz and always has a fresh detection ready.

TODO (Phase 4):
  - Optionally trigger a fresh capture-and-detect cycle on service call
    (controlled by a 'fresh_capture' bool parameter)
  - Add timeout: if detection is older than 0.5s, return detected=false
"""

import rclpy
from rclpy.node import Node
from wafer_msgs.srv import AlignWafer
from wafer_msgs.msg import WaferPose
from .camera_sim_node import CameraSimNode
from rclpy.callback_groups import ReentrantCallbackGroup
from builtin_interfaces.msg import Time


class AlignmentServiceNode(Node):
    """
    Serves the AlignWafer service using detections from the camera node.

    In the full system, this node and CameraSimNode share a single process
    via a MultiThreadedExecutor (see wafer_bringup/launch/full_system.launch.py).
    """

    def __init__(self):
        super().__init__('alignment_service')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('max_detection_age_s', 1.0)
        self.declare_parameter('confidence_threshold', 0.3)

        self._max_age = self.get_parameter('max_detection_age_s').value
        self._conf_threshold = self.get_parameter('confidence_threshold').value

        # ── Service ────────────────────────────────────────────────────
        cb_group = ReentrantCallbackGroup()
        self._srv = self.create_service(
            AlignWafer,
            '/waferflow/align_wafer',
            self._handle_align_wafer,
            callback_group=cb_group
        )

        # ── Shared camera node reference ───────────────────────────────
        # In combined deployment, the camera node is spun in the same executor.
        # For standalone testing, this can be replaced with a mock.
        self._camera_node: CameraSimNode | None = None

        self.get_logger().info('AlignmentService ready at /waferflow/align_wafer')

    def set_camera_node(self, camera_node: CameraSimNode) -> None:
        """
        Inject a reference to the running CameraSimNode.
        Called from the bringup entry point that spins both nodes.
        """
        self._camera_node = camera_node

    def _handle_align_wafer(self, request: AlignWafer.Request,
                              response: AlignWafer.Response) -> AlignWafer.Response:
        """
        Service handler: look up latest detection for the requested slot.

        Args:
            request.slot_id: e.g. 'A1'

        Returns:
            response with WaferPose filled in, or detected=False on failure.
        """
        slot_id = request.slot_id
        self.get_logger().info(f'AlignWafer requested for slot: {slot_id}')

        if self._camera_node is None:
            response.detected = False
            response.message = 'Camera node not connected to alignment service.'
            return response

        detection = self._camera_node.get_latest_detection(slot_id)

        if detection is None:
            response.detected = False
            response.message = f'No detection cached for slot {slot_id}'
            return response

        confidence = detection.get('confidence', 0.0)
        if confidence < self._conf_threshold:
            response.detected = False
            response.message = (
                f'Detection confidence {confidence:.2f} below threshold '
                f'{self._conf_threshold:.2f}'
            )
            return response

        # ── Build WaferPose response ───────────────────────────────────
        pose = WaferPose()
        pose.x = float(detection.get('dx_m', 0.0))
        pose.y = float(detection.get('dy_m', 0.0))
        pose.theta = float(detection.get('theta', 0.0))
        pose.confidence = float(confidence)
        pose.slot_id = slot_id
        pose.stamp = self.get_clock().now().to_msg()

        response.pose = pose
        response.detected = True
        response.message = 'OK'

        self.get_logger().info(
            f'AlignWafer: slot={slot_id} dx={pose.x:.4f}m '
            f'dy={pose.y:.4f}m theta={pose.theta:.4f}rad '
            f'conf={pose.confidence:.2f}'
        )
        return response


def main(args=None):
    """
    Entry point: spins both CameraSimNode and AlignmentServiceNode
    in a MultiThreadedExecutor so the service can handle requests
    while images are being processed.
    """
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()

    camera_node = CameraSimNode()
    service_node = AlignmentServiceNode()
    service_node.set_camera_node(camera_node)

    executor.add_node(camera_node)
    executor.add_node(service_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        service_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
