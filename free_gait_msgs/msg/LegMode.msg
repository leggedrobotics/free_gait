# Switch the leg mode between support leg and non-support leg mode.

# Leg name ('LF_LEG', 'RH_LEG' etc.).
string name

# If leg should be a support leg or not.
bool support_leg

# Duration of the mode switch.
# If 0, default is used.
duration duration

# If support leg: Target foothold surface normal.
# Leave empty if non-support leg or not known.
geometry_msgs/Vector3Stamped surface_normal

# Target foothold friction coefficient.
# Leave empty of no contact is expected or not known.
float64 friction_coefficient

# If pose adaptation should ignore this leg motion.
# Default is false.
bool ignore_for_pose_adaptation
