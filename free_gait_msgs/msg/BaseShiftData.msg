# Representation of base shift data.

# State type identifier (e.g. 'pre_step', 'at_step', 'post_step').
string name

# Either: Define a simple profile for the base trajectory.
free_gait_msgs/BaseShiftProfile profile

# Or: Define the entire trajectory for the base.
# The trajectory is used if it contains data, otherwise the profile is used.
trajectory_msgs/MultiDOFJointTrajectory trajectory
