# Command to let controller add auto generated base motion
# for stability and reachability of footsteps.

# Base height in control frame.
# If 0, current height of the robot is used.
float64 height

# Average linear velocity [m/s] and angular
# velocity [rad/s] of the base motion.
# If 0, default is used.
float64 average_linear_velocity
float64 average_angular_velocity

# Margin for the support polygon [m].
# If 0, default is used.
float64 support_margin