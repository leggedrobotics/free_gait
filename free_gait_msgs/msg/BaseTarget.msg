# Definition of a base target pose.

# Target pose of the base by the end of the motion.
geometry_msgs/PoseStamped target

# If the duration of the base motion should
# NOT be adapted to the duration of the step's leg.
# If no leg motion is defined in the step,
# the average velocities are used.
# Default is false.
bool ignore_timing_of_leg_motion

# Average linear velocity [m/s] and angular
# velocity [rad/s] of the base motion.
# If 0, default is used.
float64 average_linear_velocity
float64 average_angular_velocity
