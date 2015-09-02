# Representation of swing data for one leg.

# Leg type identifiers ('leftFore', 'rightHind' etc.).
string name

# Either: Set a mode for the foot/leg ('freeze', 'gravity_compensation' etc.)
string mode

# Or: Define a profile for the foot.
free_gait_msgs/SwingProfile profile

# Or: Define the entire swing trajectory for the foot.
# The trajectory is used if it contains data, otherwise the profile is used.
trajectory_msgs/MultiDOFJointTrajectory trajectory

# If a touchdown of the foot at the end of the swing is not expected.
bool no_touchdown

# Target foothold surface normal (if [0, 0, 0] set to [0, 0, 1] in world frame).
geometry_msgs/Vector3Stamped surface_normal
