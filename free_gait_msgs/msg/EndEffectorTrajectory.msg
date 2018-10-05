# Definition of a trajectory for the foot.

# Leg name ('LF_LEG', 'RH_LEG' etc.).
string name

# Trajectory for the end effector.
# Trajectory can contain transforms, twists, or accelerations,
# or combinations of these. 
trajectory_msgs/MultiDOFJointTrajectory trajectory

# Impedance control gains (k_p: position tracking, k_d: Velocity tracking, k_f: Force tracking).
geometry_msgs/Vector3Stamped[] k_p
geometry_msgs/Vector3Stamped[] k_d
geometry_msgs/Vector3Stamped[] k_f

# If true, we interpret acceleration as force
# Default is false
bool force_instead_of_acceleration

# Target surface normal.
# Leave empty if no contact is expected or not known.
geometry_msgs/Vector3Stamped surface_normal

# If contact of the end effector should be ignored.
# Default is false.
bool ignore_contact

# If pose adaptation should ignore this leg motion.
# Default is false.
bool ignore_for_pose_adaptation