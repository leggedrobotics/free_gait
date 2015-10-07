# Definition of a swing leg foot target.

# Leg type identifiers ('leftFore', 'rightHind' etc.).
string name

# Target position of the foot by the end of the profile.
geometry_msgs/PointStamped target

# Step apex swing heights in control frame.
# If 0, default is used.
float64 height

# Duration of the profile.
# If 0, default is used.
duration duration

# Type of the swing trajectory ('triangle', 'square', etc.).
# If empty, default is used.
string type