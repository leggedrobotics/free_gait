###################
# Define leg swing motions. Each leg can be defined by one
# of the following swing types:

# Step defined by foothold position and swing profile.
free_gait_msgs/Footstep[] footstep

# Move end effector to a target point, with target velocity, or acceleration.
free_gait_msgs/EndEffectorTarget[] end_effector_target

# Move the foot along a specified trajectory.
free_gait_msgs/EndEffectorTrajectory[] end_effector_trajectory

# Move the foot to a target point.
free_gait_msgs/JointTarget[] joint_target

# Defines leg motion as trajectory for each joint.
free_gait_msgs/JointTrajectory[] joint_trajectory

###################
# Define the base motion. For each state of the step (e.g. preStep,
# atStep, postStep), different base motion types can be defined:

# Specify if base motion auto generation is desired or not.
free_gait_msgs/BaseAuto[] base_auto

# Move base to a target pose.
free_gait_msgs/BaseTarget[] base_target

# Move base along a trajectory.
free_gait_msgs/BaseTrajectory[] base_trajectory
