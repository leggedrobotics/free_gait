/*
 * AdapterRosInitializerBase.hpp
 *
 *  Created on: Nov 29, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include <free_gait_core/executor/AdapterBase.hpp>

// ROS
#include <ros/node_handle.h>

namespace free_gait {

class AdapterRosInitializerBase
{
 public:
  AdapterRosInitializerBase();
  virtual ~AdapterRosInitializerBase();

  //! Initialize.
  virtual bool initializeAdapter(const ros::NodeHandle& nodeHandle, AdapterBase& adapter) const = 0;
};

} /* namespace free_gait */
