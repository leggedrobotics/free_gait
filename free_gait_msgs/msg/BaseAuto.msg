# Command to let controller add auto generated base motion
# for stability and reachability of footsteps.

# State type identifier (e.g. 'pre_step', 'at_step', 'post_step').
string state

# Ignore any base motion from auto generation.
bool ignore

# Base height in control frame.
float64 height

# Average velocity of the base motion [m/s].
# If 0, default is used.
float64 average_velocity
