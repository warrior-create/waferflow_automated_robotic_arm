#!/usr/bin/env python3
"""
tolerance_checker.py
=====================
Phase 6: Determines whether a pick-place cycle's results are within tolerance.

Used in two places:
  1. ALIGN state: pre-pick alignment check (is wafer misaligned beyond repair?)
  2. VERIFY state: post-placement quality check (did we place accurately enough?)

The tolerance thresholds come from system_params.yaml and are configured
as parameters on the PickPlaceFSM node.

In a real fab:
  - Position tolerance: typically ±0.1–0.5 mm (we use 1 mm for sim)
  - Rotation tolerance: typically ±0.1–1.0° (we use 1° = 17.5 mrad for sim)
"""


class ToleranceChecker:
    """
    Checks placement quality against configurable tolerances.

    Stateless — can be called for both alignment pre-check and placement verify.
    """

    def __init__(self, pos_tol: float = 0.001, rot_tol: float = 0.0175):
        """
        Args:
            pos_tol: maximum allowed position error [meters] (default 1 mm)
            rot_tol: maximum allowed rotation error [radians] (default ~1 degree)
        """
        self.pos_tol = pos_tol
        self.rot_tol = rot_tol

    def check(self, pos_error: float, rot_error: float) -> bool:
        """
        Check if errors are within tolerance.

        Args:
            pos_error: Euclidean position error [meters] (always non-negative)
            rot_error: absolute rotation error [radians] (always non-negative)

        Returns:
            True if BOTH position and rotation errors are within tolerance.
        """
        return pos_error <= self.pos_tol and rot_error <= self.rot_tol

    def check_alignment(self, pose) -> tuple[bool, str]:
        """
        Check pre-pick alignment quality from a WaferPose message.

        Args:
            pose: wafer_msgs/WaferPose

        Returns:
            (within_tolerance: bool, reason: str)
        """
        import math
        pos_err = math.sqrt(pose.x ** 2 + pose.y ** 2)
        rot_err = abs(pose.theta)

        reasons = []
        if pos_err > self.pos_tol:
            reasons.append(
                f'position error {pos_err*1000:.2f}mm > {self.pos_tol*1000:.2f}mm limit'
            )
        if rot_err > self.rot_tol:
            reasons.append(
                f'rotation error {math.degrees(rot_err):.2f}° > '
                f'{math.degrees(self.rot_tol):.2f}° limit'
            )

        within = len(reasons) == 0
        reason = '; '.join(reasons) if reasons else 'within tolerance'
        return within, reason

    def report(self, pos_error: float, rot_error: float) -> str:
        """Return a human-readable quality report string."""
        import math
        within = self.check(pos_error, rot_error)
        status = '✓ PASS' if within else '✗ FAIL'
        return (
            f'{status} | pos={pos_error*1000:.3f}mm '
            f'(tol={self.pos_tol*1000:.1f}mm) | '
            f'rot={math.degrees(rot_error):.3f}° '
            f'(tol={math.degrees(self.rot_tol):.1f}°)'
        )
