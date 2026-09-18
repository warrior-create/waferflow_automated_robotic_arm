# WaferPose.msg
# Represents the detected pose offset of a wafer relative to the ideal slot center.
# Positive x = wafer shifted in +X (robot frame), etc.

float64 x              # lateral offset [meters]
float64 y              # longitudinal offset [meters]
float64 theta          # rotation error [radians] — positive = CCW
float64 confidence     # detection confidence [0.0 – 1.0]
string  slot_id        # source slot identifier (e.g. "A1")
builtin_interfaces/Time stamp
