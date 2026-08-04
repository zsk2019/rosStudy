# Robot operating modes.
string MODE_IDLE="IDLE"
string MODE_RUNNING="RUNNING"
string MODE_CHARGING="CHARGING"
string MODE_ERROR="ERROR"
string MODE_AUTO="AUTO"
string MODE_EMERGENCY="EMERGENCY"

# Time when this status sample was generated.
builtin_interfaces/Time stamp

# Unique robot identifier, for example "mini_robot_01".
string robot_id

# Current operating mode. Use one of the MODE_* constants above.
string mode

# Remaining battery charge in percent, in the range [0.0, 100.0].
float32 battery

# Forward velocity in metres per second.
float32 linear_velocity

# Yaw angular velocity in radians per second.
float32 angular_velocity

# True when the emergency stop is active.
bool emergency_stop
