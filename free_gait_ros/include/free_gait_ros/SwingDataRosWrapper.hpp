/*
 * SwingDataRosWrapper.hpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/free_gait_core.hpp"

// ROS
#include <free_gait_msgs/SwingData.h>

// STD
#include <string>

namespace free_gait {

class SwingDataRosWrapper : public SwingData
{
 public:
  SwingDataRosWrapper();
  virtual ~SwingDataRosWrapper();

  /*!
   * Populate swing data from ROS swing data message.
   * @param message the swing data ROS message.
   */
  bool fromMessage(const free_gait_msgs::SwingData& message);
};

} /* namespace */
