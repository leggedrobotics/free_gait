/*
 * AdapterRos.hpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include <free_gait_ros/AdapterRosInitializerBase.hpp>

// Free Gait
#include <free_gait_core/executor/AdapterBase.hpp>

// ROS
#include <ros/node_handle.h>
#include <pluginlib/class_loader.h>

// STD
#include <memory>

namespace free_gait {

class AdapterRos
{
 public:
  AdapterRos(const ros::NodeHandle& nodeHandle);
  virtual ~AdapterRos();
  std::shared_ptr<AdapterBase> getAdapter();

 private:
  ros::NodeHandle nodeHandle_;
  pluginlib::ClassLoader<AdapterBase> adapterLoader_;
  std::shared_ptr<AdapterBase> adapter_;
  pluginlib::ClassLoader<AdapterRosInitializerBase> adapterRosInitializerLoader_;
  std::shared_ptr<AdapterRosInitializerBase> adapterRosInitializer_;
};

} /* namespace free_gait */
