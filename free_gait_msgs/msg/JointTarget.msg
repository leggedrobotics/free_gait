# Definition of trajectories for each joint of a leg.

# Leg name ('LF_LEG', 'RH_LEG' etc.).
string name

# Joint target.
trajectory_msgs/JointTrajectoryPoint target

# Target surface normal.
# Leave empty of no contact is expected or not known.
geometry_msgs/Vector3Stamped surface_normal

# If contact of the end effector should be ignored.
# Default is false.
bool ignore_contact