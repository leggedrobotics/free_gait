# Representation of swing data for one leg.

# Leg type identifiers ('leftFore', 'rightHind' etc.).
string name

# Type of the swing data (e.g. 'profile', 'foot_trajectory', 'joint_trajectory').
string type

# Either: Define a profile for the foot.
free_gait_msgs/SwingProfile profile

# Or: Define the entire swing trajectory for the foot.
trajectory_msgs/MultiDOFJointTrajectory foot_trajectory

# Or: Define the entire swing trajectory for the joints.
trajectory_msgs/JointTrajectory joint_trajectory

# If a touchdown of the foot at the end of the swing is not expected.
bool no_touchdown

# Target foothold surface normal (if [0, 0, 0] set to [0, 0, 1] in world frame).
geometry_msgs/Vector3Stamped surface_normal

# If pose adaptation should should ignore this swing data.
bool ignore_for_pose_adaptation
