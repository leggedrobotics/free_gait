/*
 * FreeGaitActionServer.hpp
 *
 *  Created on: Feb 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/free_gait_core.hpp"
#include "free_gait_ros/StepRosConverter.hpp"

// Quadruped model
#include "quadruped_model/QuadrupedModel.hpp"

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <free_gait_msgs/ExecuteStepsAction.h>

// STD
#include <string>
#include <memory>

namespace free_gait {

class FreeGaitActionServer
{
 public:
  FreeGaitActionServer(ros::NodeHandle nodeHandle, const std::string& name,
                       std::shared_ptr<Executor> executor, std::shared_ptr<AdapterBase> adapter);

  virtual ~FreeGaitActionServer();

  void initialize();
  void update();
  void shutdown();

  void goalCallback();

  void preemptCallback();

  void publishFeedback();

  void setSucceeded();

  void setPreempted();

  void setAborted();

 private:
  //! ROS nodehandle.
  ros::NodeHandle nodeHandle_;
  std::shared_ptr<free_gait::Executor> executor_;

  //! ROS converter.
  StepRosConverter rosConverter_;

  //! Action server.
  std::string name_;
  actionlib::SimpleActionServer<free_gait_msgs::ExecuteStepsAction> server_;
  free_gait_msgs::ExecuteStepsActionResult result_;

  //! True if in process of preempting.
  bool isPreempting_;
};

} /* namespace */
