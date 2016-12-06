/*
 * AdapterRosInterfaceBase.hpp
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

class AdapterRosInterfaceBase
{
 public:
  AdapterRosInterfaceBase();
  virtual ~AdapterRosInterfaceBase();
  void setNodeHandle(const ros::NodeHandle& nodeHandle);

  virtual bool subscribeToRobotState(const std::string& robotStateTopic) = 0;

  //! Update adapter.
  virtual bool initializeAdapter(AdapterBase& adapter) const = 0;
  virtual bool updateAdapterWithRobotState(AdapterBase& adapter) const = 0;

 protected:
  ros::NodeHandle nodeHandle_;

};

} /* namespace free_gait */
