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

// STD
#include <string>

namespace free_gait {

class AdapterRosInterfaceBase
{
 public:
  AdapterRosInterfaceBase();
  virtual ~AdapterRosInterfaceBase();
  void setNodeHandle(ros::NodeHandle& nodeHandle);

  virtual bool readRobotDescription();
  virtual bool subscribeToRobotState(const std::string& robotStateTopic) = 0;
  virtual void unsubscribeFromRobotState() = 0;
  virtual const std::string getRobotStateMessageType() = 0;
  virtual bool isReady() const = 0;

  //! Update adapter.
  virtual bool initializeAdapter(AdapterBase& adapter) const = 0;
  virtual bool updateAdapterWithRobotState(AdapterBase& adapter) const = 0;

 protected:
  ros::NodeHandle* nodeHandle_;
  std::string robotDescriptionUrdfString_;

};

} /* namespace free_gait */
