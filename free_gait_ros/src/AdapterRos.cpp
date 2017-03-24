/*
 * AdapterRos.cpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "free_gait_ros/AdapterRos.hpp"

namespace free_gait {

AdapterRos::AdapterRos(ros::NodeHandle& nodeHandle, const AdapterType type)
    : nodeHandle_(nodeHandle),
      adapterLoader_("free_gait_core", "free_gait::AdapterBase"),
      adapterRosInterfaceLoader_("free_gait_ros", "free_gait::AdapterRosInterfaceBase")
{
  // Load and initialize adapter.
  std::string adapterParameterName;
  if (type == AdapterType::Base) {
    adapterParameterName = "/free_gait/adapter_plugin/base";
  } else if (type == AdapterType::Preview){
    adapterParameterName = "/free_gait/adapter_plugin/preview";
  }
  std::string adapterPluginName;
  nodeHandle.getParam(adapterParameterName, adapterPluginName);
  adapter_.reset(adapterLoader_.createUnmanagedInstance(adapterPluginName));

  std::string adapterRosInterfacePluginName;
  nodeHandle.getParam("/free_gait/adapter_ros_interface_plugin", adapterRosInterfacePluginName);
  adapterRosInterface_.reset(adapterRosInterfaceLoader_.createUnmanagedInstance(adapterRosInterfacePluginName));

  adapterRosInterface_->setNodeHandle(nodeHandle_);
  adapterRosInterface_->readRobotDescription();
  adapterRosInterface_->initializeAdapter(*adapter_);
}

AdapterRos::~AdapterRos()
{
}

bool AdapterRos::subscribeToRobotState(const std::string& robotStateTopic)
{
  // Get topic name parameter.
  std::string topic(robotStateTopic);
  if (topic.empty()) {
    if (nodeHandle_.hasParam("/free_gait/robot_state")) {
      nodeHandle_.getParam("/free_gait/robot_state", topic);
    } else {
      ROS_ERROR("Did not find ROS parameter for robot state topic '/free_gait/robot_state'.");
      return false;
    }
  }

  // Subscribe.
  return adapterRosInterface_->subscribeToRobotState(topic);
}

void AdapterRos::unsubscribeFromRobotState()
{
  adapterRosInterface_->unsubscribeFromRobotState();
}

const std::string AdapterRos::getRobotStateMessageType()
{
  return adapterRosInterface_->getRobotStateMessageType();
}

bool AdapterRos::isReady() const
{
    return adapterRosInterface_->isReady();
}

bool AdapterRos::updateAdapterWithState()
{
  return adapterRosInterface_->updateAdapterWithRobotState(*adapter_);
}

const AdapterBase& AdapterRos::getAdapter() const
{
  return *adapter_;
}

AdapterBase* AdapterRos::getAdapterPtr()
{
  return adapter_.get();
}

} /* namespace free_gait */
