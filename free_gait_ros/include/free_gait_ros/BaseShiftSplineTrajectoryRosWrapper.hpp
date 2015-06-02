/*
 * BaseShiftSplineTrajectoryRosWrapper.hpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/free_gait_core.hpp"

// Curves
#include <curves_ros/RosMultiDOFJointTrajectoryInterface.hpp>

// ROS
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

// STD
#include <string>

namespace free_gait {

class BaseShiftSplineTrajectoryRosWrapper : public BaseShiftSplineTrajectory, public curves::RosMultiDOFJointTrajectoryInterface
{
 public:
  BaseShiftSplineTrajectoryRosWrapper();
  virtual ~BaseShiftSplineTrajectoryRosWrapper();

  bool fromMessage(const trajectory_msgs::MultiDOFJointTrajectory& message);

 private:
  const std::string jointName_;
};

} /* namespace */
