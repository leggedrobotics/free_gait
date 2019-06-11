# Step defined by foothold position and swing profile.

# Leg name ('LF_LEG', 'RH_LEG' etc.).
string name

# Target position of the foot by the end of the motion.
geometry_msgs/PointStamped target

# Step apex swing heights in control frame.
# If 0, default is used.
float64 profile_height

# Average velocity of the foot motion [m/s].
# If 0, default is used.
float64 average_velocity

# Type of the swing trajectory ('triangle', 'square', etc.).
# If empty, default is used.
string profile_type

# If a contact (touchdown) of the foot at the end of the swing is not expected.
# Default is false.
bool ignore_contact

# Target foothold surface normal.
# Leave empty of no contact is expected or not known.
geometry_msgs/Vector3Stamped surface_normal

# Target foothold friction coefficient.
# Leave empty of no contact is expected or not known.
float64 friction_coefficient

# If pose adaptation should ignore this leg motion.
# Default is false.
bool ignore_for_pose_adaptation
