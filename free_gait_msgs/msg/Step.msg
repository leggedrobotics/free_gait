###################
# Define leg swing motions. Each leg can be defined by one
# of the following swing types:

# Freezes all joint of a leg.
free_gait_msgs/LegFreeze[] leg_freeze

# Move the foot to a target point.
free_gait_msgs/FootTarget[] foot_target

# Move the foot along a specified trajectory.
free_gait_msgs/FootTrajectory[] foot_trajectory

# Defines leg motion as trajectory for each joint.
free_gait_msgs/JointTrajectory[] joint_trajectory

# Applies gravity compensation to a leg.
free_gait_msgs/LegFloat[] leg_float

###################
# Define the base motion. For each state of the step (e.g. preStep,
# atStep, postStep), different base motion types can be defined:

# Specify if base motion auto generation is desired or not.
free_gait_msgs/BaseAuto[] base_auto

# Move base to a target pose.
free_gait_msgs/BaseTarget[] base_target

# Move base along a trajectory.
free_gait_msgs/BaseTrajectory[] base_trajectory

# Lets the base float within the stability margins.
free_gait_msgs/BaseFloat[] base_float