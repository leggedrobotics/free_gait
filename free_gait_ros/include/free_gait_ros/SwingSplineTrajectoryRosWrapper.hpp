/*
 * SwingSplineTrajectoryRosWrapper.hpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/free_gait_core.hpp"

// Robot utils
#include <robot_utils_ros/ros_trajectory_interface/RosMultiDOFJointTrajectoryTranslationInterface.hpp>

// ROS
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

// STD
#include <string>

namespace free_gait {

class SwingSplineTrajectoryRosWrapper : public SwingSplineTrajectory, public robot_utils_ros::RosMultiDOFJointTrajectoryTranslationInterface
{
 public:
  SwingSplineTrajectoryRosWrapper();
  virtual ~SwingSplineTrajectoryRosWrapper();

  bool fromMessage(const trajectory_msgs::MultiDOFJointTrajectory& message, const std::string& jointName);
};

} /* namespace */
