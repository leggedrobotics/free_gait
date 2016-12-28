/*
 * AdapterRosInterfaceBase.cpp
 *
 *  Created on: Nov 29, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <free_gait_ros/AdapterRosInterfaceBase.hpp>

#include <pluginlib/pluginlib_exceptions.h>

namespace free_gait {

AdapterRosInterfaceBase::AdapterRosInterfaceBase()
    : nodeHandle_(NULL)
{
}

AdapterRosInterfaceBase::~AdapterRosInterfaceBase()
{
}

void AdapterRosInterfaceBase::setNodeHandle(ros::NodeHandle& nodeHandle)
{
  nodeHandle_ = &nodeHandle;
}

bool AdapterRosInterfaceBase::readRobotDescription()
{
  std::string robotDescriptionPath;
  if (nodeHandle_->hasParam("/free_gait/robot_description")) {
    nodeHandle_->getParam("/free_gait/robot_description", robotDescriptionPath);
  } else {
    throw pluginlib::PluginlibException("Did not find ROS parameter for robot description '/free_gait/robot_description'.");
  }

  if (nodeHandle_->hasParam(robotDescriptionPath)) {
    nodeHandle_->getParam(robotDescriptionPath, robotDescriptionUrdfString_);
  } else {
    throw pluginlib::PluginlibException("Did not find ROS parameter for robot description '" + robotDescriptionPath + "'.");
  }

  return true;
}

} /* namespace free_gait */
