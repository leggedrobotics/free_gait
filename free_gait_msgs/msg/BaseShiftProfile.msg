# Definition of a base shift profile.

# Either: Target pose of the base by the end of the profile.
geometry_msgs/PoseStamped target

# Or: Base height in control frame.
float64 height

# Duration of the profile.
# If 0, default is used.
duration duration

# Type of the swing trajectory ('discrete', 'linear', etc.).
# If empty, default is used.
string type
