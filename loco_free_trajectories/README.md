# Loco Joint Trajectories

Execution of joint position trajectories through the ROS interface.

## Dependencies

* [StarlETH ROS Common](https://bitbucket.org/ethz-asl-lr/starleth_ros_common), branch `feature/free_gait`.


## Build

	catkin_make -DUSE_TASK_LOCOJOINTTRAJECTORIES=ON -DUSE_TASK_LOCOJOINTTRAJECTORIES_ROS=ON


## Demo

	roslaunch starleth_gazebo starleth_minimal.launch fixed:=true
	roslaunch locomotion_controller locomotion_controller.launch

Activate `JointTrajectoriesRos` controller.

	rosrun loco_joint_trajectories_ros demo.py
