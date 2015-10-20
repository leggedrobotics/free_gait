# Definition of a base target pose.

# State name (e.g. 'pre_step', 'at_step', 'post_step').
string state

# Target pose of the base by the end of the motion.
geometry_msgs/PoseStamped target

# Average velocity of the base motion [m/s].
# If 0, default is used.
float64 average_velocity
