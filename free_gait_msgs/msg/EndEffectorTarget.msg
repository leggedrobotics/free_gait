# Definition of a target for the end effector.

# Leg name ('leftFore', 'rightHind' etc.).
string name

# Target of the end effector ('time_from_start' is ignored).
# Target can be specified as transform, twist, or acceleration,
# or a combination of these. 
trajectory_msgs/MultiDOFJointTrajectoryPoint target

# Average velocity of the end effector motion [m/s].
# Determines the duration of the motion.
# If 0, default is used.
float64 average_velocity

# Target surface normal.
# Leave empty of no contact is expected or not known.
geometry_msgs/Vector3Stamped surface_normal

# If contact of the end effector should be ignored.
# Default is false.
bool ignore_contact

# If pose adaptation should ignore this leg motion.
# Default is false.
bool ignore_for_pose_adaptation