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

# Impedance control gains (k_p: position tracking, k_d: Velocity tracking, k_f: Force tracking).
geometry_msgs/Vector3Stamped[] k_p
geometry_msgs/Vector3Stamped[] k_d
geometry_msgs/Vector3Stamped[] k_f

# Average velocity of the end effector motion [m/s].
# Determines the duration of the motion.
# If 0, default is used.
float64 average_velocity

# Duration to reach target [s].
# If set to zero, the average velocity is used to determine the duration.
float64 duration

# Target surface normal.
# Leave empty of no contact is expected or not known.
geometry_msgs/Vector3Stamped surface_normal

# If contact of the end effector should be ignored.
# Default is false.
bool ignore_contact

# If pose adaptation should ignore this leg motion.
# Default is false.
bool ignore_for_pose_adaptation
