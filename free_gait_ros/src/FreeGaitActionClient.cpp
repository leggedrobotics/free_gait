/*
 * FreeGaitActionClient.cpp
 *
 *  Created on: Oct 21, 2015
 *      Author: Georg Wiedebach, Péter Fankhauser
 */

#include <free_gait_ros/FreeGaitActionClient.hpp>

namespace free_gait {

FreeGaitActionClient::FreeGaitActionClient(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      state_(ActionState::UNINITIALIZED)
{
  // Get action server topic.
  std::string actionServerTopic;
  if (nodeHandle_.hasParam("/free_gait/action_server")) {
    nodeHandle_.getParam("/free_gait/action_server", actionServerTopic);
  } else {
    throw std::runtime_error("Did not find ROS parameter for Free Gait Action Server '/free_gait/action_server'.");
  }

  // Get preview topic.
  std::string previewTopic;
  if (nodeHandle_.hasParam("/free_gait/preview_topic")) {
    nodeHandle_.getParam("/free_gait/preview_topic", previewTopic);
  } else {
    throw std::runtime_error(
        "Did not find ROS parameter for Free Gait Preview Topic '/free_gait/preview_topic'.");
  }

  // Initialize action client.
  client_.reset(new actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>(nodeHandle, actionServerTopic));
  state_ = ActionState::DONE;

  // Initialize preview publisher.
  previewPublisher_ = nodeHandle_.advertise<free_gait_msgs::ExecuteStepsActionGoal>(previewTopic, 1, false);

  state_ = ActionState::INITIALIZED;
}

void FreeGaitActionClient::registerCallback(
    std::function<void()> activeCallback,
    std::function<void(const free_gait_msgs::ExecuteStepsFeedbackConstPtr&)> feedbackCallback,
    std::function<void(const actionlib::SimpleClientGoalState&,
                       const free_gait_msgs::ExecuteStepsResult&)> doneCallback)
{
  activeCallback_ = activeCallback;
  feedbackCallback_ = feedbackCallback;
  doneCallback_ = doneCallback;
}

void FreeGaitActionClient::sendGoal(const free_gait_msgs::ExecuteStepsGoal& goal)
{
  if (!nodeHandle_.hasParam("use_preview")) {
    ROS_ERROR("Did not find ROS parameter for 'use_preview'.");
    return;
  }

  bool usePreview;
  nodeHandle_.getParam("use_preview", usePreview);
  return sendGoal(goal, usePreview);
}

void FreeGaitActionClient::sendGoal(const free_gait_msgs::ExecuteStepsGoal& goal,
                                    const bool usePreview)
{
  if (usePreview) {
    free_gait_msgs::ExecuteStepsActionGoal actionGoal;
    actionGoal.goal = goal;
    previewPublisher_.publish(actionGoal);
    state_ = ActionState::ACTIVE;
    actionlib::SimpleClientGoalState state(actionlib::SimpleClientGoalState::ACTIVE);
    free_gait_msgs::ExecuteStepsResult result;
    result.status = free_gait_msgs::ExecuteStepsResult::RESULT_REACHED;
    doneCallback_(state, result);
  } else {
    state_ = ActionState::PENDING;
    client_->cancelAllGoals();
    client_->waitForServer();
    client_->sendGoal(goal, boost::bind(&FreeGaitActionClient::doneCallback, this, _1, _2),
                      boost::bind(&FreeGaitActionClient::activeCallback, this),
                      boost::bind(&FreeGaitActionClient::feedbackCallback, this, _1));
  }
}

void FreeGaitActionClient::waitForResult(const double timeout)
{
  client_->waitForResult(ros::Duration(timeout));
}

const FreeGaitActionClient::ActionState& FreeGaitActionClient::getState()
{
  return state_;
}

void FreeGaitActionClient::activeCallback()
{
  state_ = ActionState::ACTIVE;
  if (activeCallback_) activeCallback_();
}

void FreeGaitActionClient::feedbackCallback(
    const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback)
{
  if (feedbackCallback_) feedbackCallback_(feedback);
}

void FreeGaitActionClient::doneCallback(const actionlib::SimpleClientGoalState& state,
                                        const free_gait_msgs::ExecuteStepsResultConstPtr& result)
{
  state_ = ActionState::DONE;
  if (doneCallback_) doneCallback_(state, *result);
}

}
