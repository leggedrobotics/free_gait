# Definition of a target for the end effector.

# Leg name ('LF_LEG', 'RH_LEG' etc.).
string name

# Target of the end effector.
# Target can be specified as position, velocity, acceleration,
# force, or a combination of these.
geometry_msgs/PointStamped[] target_position
geometry_msgs/Vector3Stamped[] target_velocity
geometry_msgs/Vector3Stamped[] target_acceleration
geometry_msgs/Vector3Stamped[] target_force

# Average velocity of the end effector motion [m/s].
# Determines the duration of the motion.
# If 0, default is used.
float64 average_velocity

# Target surface normal.
# Leave empty of no contact is expected or not known.
geometry_msgs/Vector3Stamped surface_normal

# Target foothold friction coefficient.
# Leave empty of no contact is expected or not known.
float64 friction_coefficient

# If contact of the end effector should be ignored.
# Default is false.
bool ignore_contact

# If pose adaptation should ignore this leg motion.
# Default is false.
bool ignore_for_pose_adaptation
