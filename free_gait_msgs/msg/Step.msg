# The step number starting from 1, monotonically increasing during
# action, resets to 1 if robot leaves action.
uint8 step_number

# Swing data for each swing leg.
free_gait_msgs/SwingData[] swing_data

# Base shift data for different states of the step.
free_gait_msgs/BaseShiftData[] base_shift_data
