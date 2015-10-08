# Command to do gravity compensation for a leg.

# Leg identifier ('leftFore', 'rightHind' etc.).
string name

# Duration of the gravity compensation.
# If 0, infinity is used.
duration duration

# If pose should adapt for the leg.
# Default is false.
bool adapt_pose