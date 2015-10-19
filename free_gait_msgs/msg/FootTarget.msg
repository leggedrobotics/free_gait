# Definition of a swing leg foot target.

# Leg identifier ('leftFore', 'rightHind' etc.).
string name

# Target position of the foot by the end of the motion.
geometry_msgs/PointStamped target

# Step apex swing heights in control frame.
# If 0, default is used.
float64 profile_height

# Average velocity of the foot motion [m/s].
# If 0, default is used.
float64 average_velocity

# Type of the swing trajectory ('line', 'triangle', 'square', etc.).
# If empty, default is used.
string profile_type

# If a touchdown of the foot at the end of the swing is not expected.
# Default is false.
bool no_touchdown

# Target foothold surface normal.
# If empty, default is used.
geometry_msgs/Vector3Stamped surface_normal

# If pose adaptation should ignore this swing data.
# Default is false.
bool ignore_for_pose_adaptation
