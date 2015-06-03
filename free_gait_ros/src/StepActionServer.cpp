/*
 * StepActionServer.cpp
 *
 *  Created on: Feb 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_ros/StepActionServer.hpp>
#include <free_gait_ros/StepRosWrapper.hpp>

// Free Gait
#include <free_gait_core/free_gait_core.hpp>

// ROS
#include <quadruped_msgs/StepFeedback.h>
#include <quadruped_msgs/StepResult.h>

#include <iostream>

namespace free_gait {

StepActionServer::StepActionServer(ros::NodeHandle nodeHandle, const std::string& name,
                                   std::shared_ptr<free_gait::StepQueue> stepQueue,
                                   std::shared_ptr<free_gait::StepCompleter> stepCompleter,
                                   std::shared_ptr<loco::LegGroup> legs,
                                   std::shared_ptr<loco::TorsoBase> torso)
    : nodeHandle_(nodeHandle),
      name_(name),
      stepQueue_(stepQueue),
      stepCompleter_(stepCompleter),
      server_(nodeHandle_, name_, false),
      legs_(legs),
      torso_(torso),
      isPreempting_(false)
{
  server_.registerGoalCallback(boost::bind(&StepActionServer::goalCallback, this));
  server_.registerPreemptCallback(boost::bind(&StepActionServer::preemptCallback, this));
  server_.start();
  ROS_INFO_STREAM("Started " << name_ << " action server.");
}

StepActionServer::~StepActionServer()
{

}

void StepActionServer::update()
{
  if (!server_.isActive()) return;
  if (stepQueue_->empty()) {
    // Succeeded.
    if (isPreempting_) {
      // Preempted.
      setPreempted();
    } else {
      setSucceeded();
    }
  } else {
    // Ongoing.
    publishFeedback();
  }
}

void StepActionServer::goalCallback()
{
  ROS_DEBUG("Received goal for StepAction.");
//  if (server_.isActive()) server_.setRejected();

  for (auto& stepMessage : server_.acceptNewGoal()->steps) {
    StepRosWrapper step(legs_, torso_);
    step.fromMessage(stepMessage);
    stepCompleter_->complete(step);
    stepQueue_->add(step);
  }
}

void StepActionServer::preemptCallback()
{
  ROS_INFO("StepAction is requested to preempt.");
  if (stepQueue_->empty()) return;
  if (stepQueue_->size() <= 1) return;
  stepQueue_->clearNextSteps();
  isPreempting_ = true;
}

void StepActionServer::publishFeedback()
{
  quadruped_msgs::StepFeedback feedback;
  if (stepQueue_->empty()) return;
  auto& step = stepQueue_->getCurrentStep();
  feedback.step_number = step.getStepNumber();

  if (step.checkStatus() == false) {
    feedback.status = quadruped_msgs::StepFeedback::PROGRESS_PAUSED;
    feedback.description = "Paused.";
  } else {
    switch (step.getState()) {
      feedback.status = quadruped_msgs::StepFeedback::PROGRESS_EXECUTING;
      case Step::State::PreStep:
        feedback.description = "Pre step.";
        break;
      case Step::State::AtStep:
        feedback.description = "At step.";
        break;
      case Step::State::PostStep:
        feedback.description = "Post step.";
        break;
      default:
        feedback.status = quadruped_msgs::StepFeedback::PROGRESS_UNKNOWN;
        feedback.description = "Unknown.";
        break;
    }
  }

  feedback.duration = ros::Duration(step.getTotalDuration());
  feedback.phase = step.getTotalPhase();
  for (const auto& stepData : step.getSwingData())
    feedback.swing_leg_names.push_back(stepData.first);

  server_.publishFeedback(feedback);
}

void StepActionServer::setSucceeded()
{
  ROS_INFO("StepAction succeeded.");
  quadruped_msgs::StepResult result;
  result.status = quadruped_msgs::StepResult::RESULT_REACHED;
  server_.setSucceeded(result, "Step action has been reached.");
}

void StepActionServer::setPreempted()
{
  ROS_INFO("StepAction preempted.");
  quadruped_msgs::StepResult result;
  result.status = quadruped_msgs::StepResult::RESULT_FAILED;
  server_.setPreempted(result, "Step action has been preempted.");
  isPreempting_ = false;
}

void StepActionServer::setAborted()
{
  ROS_INFO("StepAction aborted.");
  quadruped_msgs::StepResult result;
  result.status = quadruped_msgs::StepResult::RESULT_FAILED;
  server_.setAborted(result, "Step action has failed.");
}

} /* namespace */
