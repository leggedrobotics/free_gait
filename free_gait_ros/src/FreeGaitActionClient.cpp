/*
 * FreeGaitActionClient.cpp
 *
 *  Created on: Oct 21, 2015
 *      Author: Georg Wiedebach, Péter Fankhauser
 */

#include <free_gait_ros/FreeGaitActionClient.hpp>

namespace free_gait {

FreeGaitActionClient::FreeGaitActionClient(ros::NodeHandle& nodeHandle,
                                           const std::string& name)
    : nodeHandle_(nodeHandle),
      client_(nodeHandle, name)
{
  state_ = ActionState::DONE;
}

void FreeGaitActionClient::sendGoal()
{
  state_ = ActionState::PENDING;
  client_.cancelAllGoals();
  client_.waitForServer();
  client_.sendGoal(goal_, boost::bind(&FreeGaitActionClient::doneCallback_, this, _1, _2),
                    boost::bind(&FreeGaitActionClient::activeCallback_, this),
                    boost::bind(&FreeGaitActionClient::feedbackCallback_, this, _1));
}

void FreeGaitActionClient::waitForResult(double timeout)
{
  client_.waitForResult(ros::Duration(timeout));
}

const free_gait_msgs::ExecuteStepsResult& FreeGaitActionClient::getResult()
{
  return result_;
}

FreeGaitActionClient::ActionState FreeGaitActionClient::getState()
{
  return state_;
}

void FreeGaitActionClient::doneCallback_(const actionlib::SimpleClientGoalState& state,
                               const free_gait_msgs::ExecuteStepsResultConstPtr& result)
{
  state_ = ActionState::DONE;
  result_ = *result;
  doneCallback(state, result);
}

void FreeGaitActionClient::activeCallback_()
{
  state_ = ActionState::ACTIVE;
  activeCallback();
}

void FreeGaitActionClient::feedbackCallback_(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback)
{
  feedbackCallback(feedback);
}

}
