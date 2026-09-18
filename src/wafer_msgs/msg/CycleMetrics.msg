# CycleMetrics.msg
# Trajectory quality metrics for one pick-place cycle.
# peak_jerk is the key particle-risk proxy — high jerk = more vibration = particle generation.

float64 cycle_time       # actual total elapsed time [seconds]
float64 peak_jerk        # maximum joint-space jerk norm over the cycle [rad/s³]
float64 duration         # trajectory execution time (excludes vision/service latency) [s]
bool    success          # did the cycle complete without reject/re-pick?
int32   cycle_index      # sequential cycle counter
string  phase_failed     # non-empty if cycle failed: one of ALIGN/PICK/TRANSIT/PLACE/VERIFY
builtin_interfaces/Time stamp
