# Definition of a trajectory for the foot.

# Leg name ('leftFore', 'rightHind' etc.).
string name

# Trajectory for the end effector.
# Trajectory can contain transforms, twists, or accelerations,
# or combinations of these. 
trajectory_msgs/MultiDOFJointTrajectory trajectory

# Target surface normal.
# Leave empty of no contact is expected or not known.
geometry_msgs/Vector3Stamped surface_normal

# If contact of the end effector should be ignored.
# Default is false.
bool ignore_contact

# If pose adaptation should ignore this leg motion.
# Default is false.
bool ignore_for_pose_adaptation