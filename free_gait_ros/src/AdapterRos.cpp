/*
 * AdapterRos.cpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "free_gait_ros/AdapterRos.hpp"

namespace free_gait {

AdapterRos::AdapterRos(const ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      adapterLoader_("free_gait_core", "free_gait::AdapterBase"),
      adapterRosInitializerLoader_("free_gait_ros", "free_gait::AdapterRosInitializerBase")
{
  // Load and initialize adapter.
  std::string adapterPluginName;
  nodeHandle.getParam("/free_gait/adapter_plugin", adapterPluginName);
  try {
    adapter_.reset(adapterLoader_.createUnmanagedInstance(adapterPluginName));
  } catch (pluginlib::PluginlibException& ex) {
    ROS_ERROR("The Free Gait adapter plugin failed to load. Error: %s", ex.what());
    return;
  }

  std::string adapterRosInitializerPluginName;
  nodeHandle.getParam("/free_gait/adapter_ros_initializer_plugin", adapterRosInitializerPluginName);
  try {
    adapterRosInitializer_.reset(adapterRosInitializerLoader_.createUnmanagedInstance(adapterRosInitializerPluginName));
  } catch (pluginlib::PluginlibException& ex) {
    ROS_ERROR("The Free Gait adapter ROS initializer plugin failed to load. Error: %s", ex.what());
    return;
  }

  adapterRosInitializer_->initializeAdapter(nodeHandle_, *adapter_);
}

AdapterRos::~AdapterRos()
{
}

std::shared_ptr<AdapterBase> AdapterRos::getAdapter()
{
  return adapter_;
}

} /* namespace free_gait */
