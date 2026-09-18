#!/usr/bin/env python3
"""
camera_sim_node.py
==================
Phase 4: Subscribes to the Gazebo camera topic and forwards images
for processing by disc_detector.py.

ROS2 Topics (subscribes):
  /waferflow/camera/image_raw  [sensor_msgs/Image]
  /waferflow/camera/camera_info [sensor_msgs/CameraInfo]

ROS2 Topics (publishes, for debugging):
  /waferflow/camera/debug_image [sensor_msgs/Image]  — annotated detection result

The node bridges raw sensor images into the detection pipeline and
caches the camera_info (intrinsics) needed for metric-space pose estimation.

TODO (Phase 4):
  - Connect to disc_detector.py for actual detection
  - Publish debug image with detected contours/center overlaid
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from .disc_detector import DiscDetector


class CameraSimNode(Node):
    """
    Camera simulation node.

    Subscribes to raw images from Gazebo's camera plugin, runs the
    DiscDetector on each frame, and publishes annotated debug images.
    """

    def __init__(self):
        super().__init__('camera_sim_node')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('debug_publish', True)
        self.declare_parameter('detection_mode', 'contour')  # 'contour' | 'aruco'
        self.declare_parameter('ideal_slot_centers', [
            # [x_pixel, y_pixel] for each slot A1-A5 in camera image frame
            # TODO: calibrate from camera extrinsics + slot world positions
            320, 96,
            320, 168,
            320, 240,
            320, 312,
            320, 384,
        ])

        debug_publish = self.get_parameter('debug_publish').value
        detection_mode = self.get_parameter('detection_mode').value

        # ── Subscribers ────────────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image, '/waferflow/camera/image_raw',
            self._image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/waferflow/camera/camera_info',
            self._camera_info_callback, 10
        )

        # ── Publishers ────────────────────────────────────────────────
        if debug_publish:
            self.debug_pub = self.create_publisher(
                Image, '/waferflow/camera/debug_image', 10
            )
        else:
            self.debug_pub = None

        # ── State ──────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.camera_info: CameraInfo | None = None
        self.detector = DiscDetector(mode=detection_mode)
        self.latest_detections: dict = {}  # slot_id -> detection result

        self.get_logger().info(
            f'CameraSimNode started. Mode: {detection_mode}'
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Cache camera intrinsics for metric-space pose estimation."""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Camera intrinsics received and cached.')

    def _image_callback(self, msg: Image) -> None:
        """
        Process each incoming camera frame.

        TODO (Phase 4):
          1. Convert ROS Image → OpenCV BGR image using cv_bridge
          2. Run disc_detector on the frame
          3. Cache latest detections in self.latest_detections
          4. Publish debug image with contours/center overlaid
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Run detection on the full image
        detections = self.detector.detect_all_wafers(cv_image, self.camera_info)
        self.latest_detections = detections

        # Publish debug image if enabled
        if self.debug_pub is not None:
            debug_img = self._draw_detections(cv_image, detections)
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f'Debug publish error: {e}')

    def _draw_detections(self, image: np.ndarray, detections: dict) -> np.ndarray:
        """
        Draw detected wafer centers and rotation arrows on the debug image.

        Args:
            image: BGR OpenCV image
            detections: dict mapping slot_id -> {center_px, center_py, theta, confidence}

        Returns:
            Annotated BGR image
        """
        annotated = image.copy()
        for slot_id, det in detections.items():
            if det is None:
                continue
            cx, cy = int(det.get('center_px', 0)), int(det.get('center_py', 0))
            theta = det.get('theta', 0.0)
            conf = det.get('confidence', 0.0)

            # Draw detected center
            cv2.circle(annotated, (cx, cy), 5, (0, 255, 0), -1)

            # Draw rotation arrow
            arrow_len = 40
            ex = int(cx + arrow_len * np.cos(theta))
            ey = int(cy - arrow_len * np.sin(theta))
            cv2.arrowedLine(annotated, (cx, cy), (ex, ey), (255, 100, 0), 2)

            # Label
            cv2.putText(annotated, f'{slot_id} c={conf:.2f}',
                        (cx - 30, cy - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        return annotated

    def get_latest_detection(self, slot_id: str) -> dict | None:
        """Called by alignment_service to get the latest detection for a slot."""
        return self.latest_detections.get(slot_id)


def main(args=None):
    rclpy.init(args=args)
    node = CameraSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
