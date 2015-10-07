# Definition of a base motion trajectory.

# State type identifier (e.g. 'pre_step', 'at_step', 'post_step').
string name

# Enforce that no automatic base shifting is added for this state.
bool ignore

# Type of the base shift (e.g. 'profile', 'trajectory', 'ignore').
string type

# Either: Define a simple profile for the base trajectory.
free_gait_msgs/BaseShiftProfile profile

# Or: Define the entire trajectory for the base.
trajectory_msgs/MultiDOFJointTrajectory trajectory
