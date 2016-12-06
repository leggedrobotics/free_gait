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

} /* namespace free_gait */

