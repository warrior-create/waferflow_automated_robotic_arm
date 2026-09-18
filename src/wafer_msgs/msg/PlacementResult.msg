# PlacementResult.msg
# Per-cycle placement quality metrics published after each pick-place cycle.

float64 position_error       # Euclidean distance from commanded target [meters]
float64 orientation_error    # absolute rotation error at placement [radians]
bool    success              # true if both errors below tolerance thresholds
string  slot_id              # destination slot identifier
int32   cycle_index          # sequential cycle counter within a sweep run
float64 cycle_time_target    # requested cycle time [seconds]
builtin_interfaces/Time stamp
