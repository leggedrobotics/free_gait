# Loco Free Gait

A free gait implementation for loco.

## Build status

[![Build Status](http://192.168.0.40:8080/buildStatus/icon?job=free_gait)](http://192.168.0.40:8080/job/free_gait/)

## Dependencies

## Build

	catkin_make -DUSE_TASK_LOCOFREEGAIT=ON -DUSE_TASK_LOCOFREEGAIT_ROS=ON

## Unit Tests
	catkin build free_gait_core --no-deps --verbose --catkin-make-args run_tests