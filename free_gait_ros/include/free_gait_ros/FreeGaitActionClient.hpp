/*
 * FreeGaitActionClient.hpp
 *
 *  Created on: Oct 21, 2015
 *      Author: Péter Fankhauser, Georg Wiedebach
 */

#pragma once

// Free Gait
#include <free_gait_core/free_gait_core.hpp>
#include <free_gait_msgs/ExecuteStepsAction.h>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>

// STD
#include <memory>
#include <functional>

namespace free_gait {

class FreeGaitActionClient
{
 public:
  enum ActionState
  {
    ERROR,
    UNINITIALIZED,
    INITIALIZED,
    PENDING,
    ACTIVE,
    IDLE,
    DONE
  };

  FreeGaitActionClient(ros::NodeHandle& nodeHandle);
  virtual ~FreeGaitActionClient() {};

  void registerCallback(std::function<void()> activeCallback = nullptr,
                        std::function<void(const free_gait_msgs::ExecuteStepsFeedbackConstPtr&)> feedbackCallback = nullptr,
                        std::function<void(const actionlib::SimpleClientGoalState&,
                                           const free_gait_msgs::ExecuteStepsResult&)> doneCallback = nullptr);
  void sendGoal(const free_gait_msgs::ExecuteStepsGoal& goal);
  void sendGoal(const free_gait_msgs::ExecuteStepsGoal& goal, const bool usePreview);
  void waitForResult(const double timeout = 1e6);  // TODO
  const ActionState& getState();

 private:
  void activeCallback();
  void feedbackCallback(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback);
  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const free_gait_msgs::ExecuteStepsResultConstPtr& result);

  ros::NodeHandle& nodeHandle_;
  std::function<void()> activeCallback_;
  std::function<void(const free_gait_msgs::ExecuteStepsFeedbackConstPtr&)> feedbackCallback_;
  std::function<void(const actionlib::SimpleClientGoalState&,
                     const free_gait_msgs::ExecuteStepsResult&)> doneCallback_;
  std::unique_ptr<actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>> client_;
  ros::Publisher previewPublisher_;
  ActionState state_;
};

}
