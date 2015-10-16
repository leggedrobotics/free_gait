/*
 * SwingJointTrajectoryRosWrapper.hpp
 *
 *  Created on: Sep 2, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/free_gait_core.hpp"

// ROS
#include <trajectory_msgs/JointTrajectory.h>

// STD
#include <string>

namespace free_gait {

class SwingJointTrajectoryRosWrapper :
    public SwingJointTrajectory
{
 public:
  SwingJointTrajectoryRosWrapper();
  virtual ~SwingJointTrajectoryRosWrapper();

  bool fromMessage(const trajectory_msgs::JointTrajectory& message);
};

} /* namespace */
