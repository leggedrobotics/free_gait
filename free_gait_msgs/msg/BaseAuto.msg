# Command to let controller add auto generated base motion
# for stability and reachability of footsteps.

# State type identifier (e.g. 'pre_step', 'at_step', 'post_step').
string name

# Add auto generated motion or ignore auto generation.
bool active

# Velocity of the base motion [m/s].
# If 0, default is used.
float64 velocity