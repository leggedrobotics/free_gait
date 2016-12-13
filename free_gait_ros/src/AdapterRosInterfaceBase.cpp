/*
 * AdapterRosInterfaceBase.cpp
 *
 *  Created on: Nov 29, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <free_gait_ros/AdapterRosInterfaceBase.hpp>

namespace free_gait {

AdapterRosInterfaceBase::AdapterRosInterfaceBase()
{
}

AdapterRosInterfaceBase::~AdapterRosInterfaceBase()
{
}

void AdapterRosInterfaceBase::setNodeHandle(const ros::NodeHandle& nodeHandle)
{
  nodeHandle_ = nodeHandle;
}

bool AdapterRosInterfaceBase::readRobotDescription()
{
  std::string robotDescriptionPath;
  if (nodeHandle_.hasParam("/free_gait/robot_description")) {
    nodeHandle_.getParam("/free_gait/robot_description", robotDescriptionPath);
  } else {
    ROS_ERROR("Did not find ROS parameter for robot description '/free_gait/robot_description'.");
    return false;
  }

  if (nodeHandle_.hasParam(robotDescriptionPath)) {
    nodeHandle_.getParam(robotDescriptionPath, robotDescriptionUrdfString_);
  } else {
    ROS_ERROR_STREAM("Did not find ROS parameter for robot description '" << robotDescriptionPath << ".");
    return false;
  }

  return true;
}

} /* namespace free_gait */
