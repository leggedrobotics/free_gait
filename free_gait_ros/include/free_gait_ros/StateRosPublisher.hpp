/*
 * StateRosPublisher.hpp
 *
 *  Created on: Dec 6, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include <free_gait_core/free_gait_core.hpp>

// ROS
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf2_ros/transform_broadcaster.h>

// STD
#include <memory>
#include <string>

namespace free_gait {

class StateRosPublisher
{
 public:
  StateRosPublisher(ros::NodeHandle& nodeHandle, AdapterBase& adapter);
  virtual ~StateRosPublisher();
  StateRosPublisher(const StateRosPublisher& other);

  void setTfPrefix(const std::string tfPrefix);
  bool publish(const State& state);

 private:
  bool initializeRobotStatePublisher();

  ros::NodeHandle& nodeHandle_;
  std::string tfPrefix_;
  std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisher_;
  AdapterBase& adapter_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;
};

} /* namespace free_gait */
