/*
 * FreeGaitActionClient.hpp
 *
 *  Created on: Oct 21, 2015
 *      Author: Péter Fankhauser, Georg Wiedebach
 */

#pragma once

#include "free_gait_ros/AdapterRos.hpp"
#include "free_gait_ros/StepFrameConverter.hpp"
#include "free_gait_ros/StepRosConverter.hpp"

// Free Gait
#include <free_gait_core/free_gait_core.hpp>
#include <free_gait_msgs/ExecuteStepsAction.h>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>

// STD
#include <memory>

namespace free_gait {

class FreeGaitActionClient
{
 public:
  enum ActionState
  {
    PENDING,
    ACTIVE,
    DONE
  };

  FreeGaitActionClient(ros::NodeHandle& nodeHandle, const std::string& name = "");
  virtual ~FreeGaitActionClient() {};

  void sendGoal(const free_gait_msgs::ExecuteStepsGoal& goal);
  void waitForResult(double timeout = 1e6);  // TODO
  const free_gait_msgs::ExecuteStepsResult& getResult();
  ActionState getState();

 protected:
  virtual void activeCallback() {};
  virtual void feedbackCallback(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback) {};
  virtual void doneCallback(const actionlib::SimpleClientGoalState& state,
                            const free_gait_msgs::ExecuteStepsResultConstPtr& result) {};

  AdapterRos adapterRos_;
  std::unique_ptr<free_gait::StepRosConverter> rosConverter_;
  std::unique_ptr<free_gait::StepFrameConverter> frameConverter_;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;

 private:
  void activeCallback_();
  void feedbackCallback_(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback);
  void doneCallback_(const actionlib::SimpleClientGoalState& state,
                     const free_gait_msgs::ExecuteStepsResultConstPtr& result);

  ros::NodeHandle& nodeHandle_;
  std::unique_ptr<actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>> client_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;
  ActionState state_;
  free_gait_msgs::ExecuteStepsResult result_;
};

}
