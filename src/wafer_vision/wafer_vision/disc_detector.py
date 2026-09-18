#!/usr/bin/env python3
"""
disc_detector.py
================
Phase 4: OpenCV-based wafer disc pose estimator.

Two detection modes:
  1. 'contour' — threshold + findContours + minAreaRect
     Works when the wafer is visually distinct (grey disc on dark background).
     Detects center (dx, dy) offset and rotation (dtheta) from ideal slot center.

  2. 'aruco' — ArUco marker detection (cv2.aruco)
     More robust when lighting varies. Requires a printed ArUco marker
     stuck to the wafer surface (also visible in the Gazebo model via
     a texture — see TODO below). Uses solvePnP for full 6-DOF pose.

The output of both modes is a unified dict:
  {
    'center_px': float,   # x pixel coordinate of detected center
    'center_py': float,   # y pixel coordinate of detected center
    'dx_m': float,        # lateral offset in meters (world frame)
    'dy_m': float,        # longitudinal offset in meters
    'theta': float,       # rotation error in radians
    'confidence': float,  # 0.0 – 1.0
  }

TODO (Phase 4):
  - Tune HSV/threshold parameters for Gazebo's lighting
  - Implement ArUco mode with cv2.aruco.DetectorParameters
  - Add solvePnP for full 6-DOF pose estimation in ArUco mode
  - Calibrate pixel-to-meter conversion using camera_info intrinsics
"""

import cv2
import numpy as np
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class SlotConfig:
    """Ideal center and region of interest for a cassette slot."""
    slot_id: str
    ideal_px: float    # ideal center x in image [pixels]
    ideal_py: float    # ideal center y in image [pixels]
    # ROI: crop around ideal center to reduce false positives
    roi_half_w: int = 80
    roi_half_h: int = 80


# Default slot configs — adjust after camera calibration
DEFAULT_SLOTS = [
    SlotConfig('A1', ideal_px=320, ideal_py=96),
    SlotConfig('A2', ideal_px=320, ideal_py=168),
    SlotConfig('A3', ideal_px=320, ideal_py=240),
    SlotConfig('A4', ideal_px=320, ideal_py=312),
    SlotConfig('A5', ideal_px=320, ideal_py=384),
]

# ARUCO dictionary — use 4x4 250 for compact markers
ARUCO_DICT = cv2.aruco.DICT_4X4_250


class DiscDetector:
    """
    Detects wafer disc center and rotation in overhead camera images.

    Supports two modes: 'contour' (faster, simpler) and 'aruco' (more robust).
    Converts pixel-space measurements to metric offsets using camera intrinsics.
    """

    def __init__(self, mode: str = 'contour',
                 slot_configs: list[SlotConfig] = None):
        """
        Args:
            mode: 'contour' or 'aruco'
            slot_configs: list of SlotConfig for each cassette slot
        """
        self.mode = mode
        self.slots = {s.slot_id: s for s in (slot_configs or DEFAULT_SLOTS)}

        if mode == 'aruco':
            self._aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
            self._aruco_params = cv2.aruco.DetectorParameters()
            self._aruco_detector = cv2.aruco.ArucoDetector(
                self._aruco_dict, self._aruco_params
            )

        # Pixel → meter scale (set from camera_info in calibrate_from_camera_info)
        self._px_per_meter: float = 1000.0  # TODO: calibrate

    # ──────────────────────────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────────────────────────

    def calibrate_from_camera_info(self, camera_info) -> None:
        """
        Extract pixel-to-meter scaling from camera intrinsics + known camera height.

        The camera is mounted at a known height above the slot (from URDF).
        f_x from camera_info.k[0] gives the focal length in pixels.
        scale = f_x / camera_height  [px/m]

        TODO (Phase 4): Implement this properly once camera height is confirmed.
        """
        if camera_info is not None:
            fx = camera_info.k[0]
            camera_height = 0.60  # meters above slot (see scara_arm.gazebo.xacro)
            self._px_per_meter = fx / camera_height

    def detect_all_wafers(self, image: np.ndarray, camera_info=None) -> dict:
        """
        Run detection on all configured slots.

        Args:
            image: BGR OpenCV image from camera
            camera_info: ROS CameraInfo message (used for metric conversion)

        Returns:
            dict mapping slot_id -> detection result dict (or None if not detected)
        """
        if camera_info is not None:
            self.calibrate_from_camera_info(camera_info)

        results = {}
        for slot_id, slot in self.slots.items():
            results[slot_id] = self._detect_single_slot(image, slot)
        return results

    def detect_wafer(self, image: np.ndarray, slot_id: str,
                     camera_info=None) -> Optional[dict]:
        """
        Detect a single wafer in the specified slot.

        Args:
            image: BGR OpenCV image
            slot_id: e.g. 'A1'
            camera_info: ROS CameraInfo (optional)

        Returns:
            detection dict or None if no wafer found
        """
        if camera_info is not None:
            self.calibrate_from_camera_info(camera_info)

        slot = self.slots.get(slot_id)
        if slot is None:
            raise ValueError(f'Unknown slot: {slot_id}')
        return self._detect_single_slot(image, slot)

    # ──────────────────────────────────────────────────────────────────
    # Internal: contour mode
    # ──────────────────────────────────────────────────────────────────

    def _detect_single_slot(self, image: np.ndarray,
                             slot: SlotConfig) -> Optional[dict]:
        """Route to appropriate detection backend."""
        if self.mode == 'aruco':
            return self._detect_aruco(image, slot)
        else:
            return self._detect_contour(image, slot)

    def _detect_contour(self, image: np.ndarray,
                         slot: SlotConfig) -> Optional[dict]:
        """
        Contour-based detection using thresholding + HoughCircles.
        """
        roi, offset_x, offset_y = self._crop_roi(image, slot)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        
        # We expect a circle of radius ~ 150px (0.15m wafer radius)
        # We need to compute expected radius based on px_per_meter. Wafer is 0.300m dia -> 0.150m radius.
        expected_r = int(0.150 * self._px_per_meter)
        
        # HoughCircles to find the wafer disc
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=100,
                                   param1=50, param2=30, 
                                   minRadius=int(expected_r * 0.8), 
                                   maxRadius=int(expected_r * 1.2))
                                   
        if circles is None:
            return None
            
        circles = np.uint16(np.around(circles))
        best_circle = circles[0, 0] # Take the strongest circle
        cx_roi, cy_roi, r = best_circle[0], best_circle[1], best_circle[2]

        # Convert ROI-local coords → full image coords
        cx_img = cx_roi + offset_x
        cy_img = cy_roi + offset_y

        # Pixel offset from ideal center
        dpx = cx_img - slot.ideal_px
        dpy = cy_img - slot.ideal_py

        # Metric conversion
        dx_m = dpx / self._px_per_meter
        dy_m = dpy / self._px_per_meter

        # Rotation: Cannot easily detect rotation from a featureless disc with HoughCircles.
        # We will assume 0 rotation error for this mode unless we detect the notch.
        theta_rad = 0.0

        # Confidence based on radius match
        confidence = float(np.clip(1.0 - abs(r - expected_r) / float(expected_r), 0.0, 1.0))

        return {
            'center_px': float(cx_img),
            'center_py': float(cy_img),
            'dx_m': float(dx_m),
            'dy_m': float(dy_m),
            'theta': float(theta_rad),
            'confidence': confidence,
        }

    def _detect_aruco(self, image: np.ndarray,
                       slot: SlotConfig) -> Optional[dict]:
        """
        ArUco-marker based detection.

        Expects a 4x4 ArUco marker on the wafer surface (marker ID = slot index).
        Uses detectMarkers → solvePnP for full 6-DOF pose.

        TODO (Phase 4):
          - Add marker texture to wafer_disc SDF model
          - Implement solvePnP with camera_info K matrix
          - Map marker pose → (dx, dy, theta) offset from ideal center
        """
        corners, ids, _ = self._aruco_detector.detectMarkers(image)
        if ids is None or len(ids) == 0:
            return None

        # TODO: match marker ID to slot, compute pose, return dict
        # Placeholder return:
        return {
            'center_px': slot.ideal_px,
            'center_py': slot.ideal_py,
            'dx_m': 0.0,
            'dy_m': 0.0,
            'theta': 0.0,
            'confidence': 0.5,
        }

    # ──────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────

    def _crop_roi(self, image: np.ndarray,
                   slot: SlotConfig) -> tuple[np.ndarray, int, int]:
        """
        Crop a Region of Interest around the slot's ideal center.

        Returns (roi, x_offset, y_offset) where offsets convert ROI-local
        coordinates back to full-image coordinates.
        """
        h_img, w_img = image.shape[:2]
        x1 = max(0, int(slot.ideal_px) - slot.roi_half_w)
        y1 = max(0, int(slot.ideal_py) - slot.roi_half_h)
        x2 = min(w_img, int(slot.ideal_px) + slot.roi_half_w)
        y2 = min(h_img, int(slot.ideal_py) + slot.roi_half_h)
        roi = image[y1:y2, x1:x2]
        return roi, x1, y1
