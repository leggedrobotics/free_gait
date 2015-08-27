/*
 * StepActionServer.hpp
 *
 *  Created on: Feb 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/free_gait_core.hpp"

// Loco
#include "loco/common/LegGroup.hpp"

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <free_gait_msgs/StepAction.h>

// STD
#include <string>
#include <memory>

namespace free_gait {

class StepActionServer
{
 public:
  StepActionServer(ros::NodeHandle nodeHandle, const std::string& name,
                   std::shared_ptr<StepQueue> stepQueue,
                   std::shared_ptr<StepCompleter> stepCompleter,
                   std::shared_ptr<loco::LegGroup> legs);
  virtual ~StepActionServer();

  void update();

  void goalCallback();

  void preemptCallback();

  void publishFeedback();

  void setSucceeded();

  void setPreempted();

  void setAborted();

 private:

  //! ROS nodehandle.
  ros::NodeHandle nodeHandle_;
  std::shared_ptr<free_gait::StepQueue> stepQueue_;

  //! Step completion.
  std::shared_ptr<free_gait::StepCompleter> stepCompleter_;

  //! Common loco leg group class.
  std::shared_ptr<loco::LegGroup> legs_;

  std::string name_;
  actionlib::SimpleActionServer<free_gait_msgs::StepAction> server_;
  free_gait_msgs::StepActionResult result_;

  //! True if in process of preempting.
  bool isPreempting_;
};

} /* namespace */
