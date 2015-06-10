# Loco Free Gait

A free gait implementation for loco.

## Dependencies

* [Loco Crawling](https://bitbucket.org/ethz-asl-lr/loco_crawling/), branch `feature/free_gait`,
* [StarlETH ROS Common](https://bitbucket.org/ethz-asl-lr/starleth_ros_common), branch `feature/free_gait`.


## Build

	catkin_make -DUSE_TASK_LOCOFREEGAIT=ON -DUSE_TASK_LOCOFREEGAIT_ROS=ON

## Unit Tests
	catkin build free_gait_core --no-deps --verbose --catkin-make-args run_tests
