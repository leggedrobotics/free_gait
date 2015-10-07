# Swing data for each swing leg.
free_gait_msgs/SwingData[] leg_
free_gait_msgs/FootTarget[] foot_target
free_gait_msgs/FootTrajectory[] foot_trajectory
free_gait_msgs/LegJointTrajectory[] joint_trajectory

# Enforce that no automatic base shifting is added (all states).
bool ignore_all_base_shift

# Base shift data for different states of the step.
free_gait_msgs/BaseShiftIgnore[] base_shift_ignore
free_gait_msgs/BaseHeight[] base_height
free_gait_msgs/BaseTarget[] base_target
free_gait_msgs/BaseTrajectory[] base_trajectory
