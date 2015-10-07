# Definition of trajectories for each joint.

# Leg identifier ('leftFore', 'rightHind' etc.).
string name

# Joint trajectories.
trajectory_msgs/JointTrajectory trajectory

# If a touchdown of the foot at the end of the swing is not expected.
# Default is false.
bool no_touchdown

# Target foothold surface normal.
# If empty, default is used.
geometry_msgs/Vector3Stamped surface_normal

# If pose adaptation should ignore this swing data.
# Default is false.
bool ignore_for_pose_adaptation