/*
 * FreeGaitActionServer.cpp
 *
 *  Created on: Feb 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_ros/FreeGaitActionServer.hpp>
#include <free_gait_core/free_gait_core.hpp>

// ROS
#include <free_gait_msgs/ExecuteStepsFeedback.h>
#include <free_gait_msgs/ExecuteStepsResult.h>

#include <iostream>

namespace free_gait {

FreeGaitActionServer::FreeGaitActionServer(ros::NodeHandle nodeHandle, const std::string& name,
                                   std::shared_ptr<free_gait::StepQueue> stepQueue,
                                   std::shared_ptr<free_gait::StepCompleter> stepCompleter,
                                   std::shared_ptr<quadruped_model::QuadrupedModel> quadrupedModel)
    : nodeHandle_(nodeHandle),
      name_(name),
      stepQueue_(stepQueue),
      rosConverter_(quadrupedModel),
      server_(nodeHandle_, name_, false),
      quadrupedModel_(quadrupedModel),
      isPreempting_(false)
{
}

FreeGaitActionServer::~FreeGaitActionServer()
{
}

void FreeGaitActionServer::initialize()
{
  server_.registerGoalCallback(boost::bind(&FreeGaitActionServer::goalCallback, this));
  server_.registerPreemptCallback(boost::bind(&FreeGaitActionServer::preemptCallback, this));
  server_.start();
  ROS_INFO_STREAM("Started " << name_ << " action server.");
}

void FreeGaitActionServer::update()
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

void FreeGaitActionServer::shutdown()
{
  server_.shutdown();
}

void FreeGaitActionServer::goalCallback()
{
  ROS_DEBUG("Received goal for StepAction.");
//  if (server_.isActive()) server_.setRejected();

  const auto goal = server_.acceptNewGoal();

  for (auto& stepMessage : goal->steps) {
    Step step;
    rosConverter_.fromMessage(stepMessage, step);
    stepQueue_->add(step);
  }
}

void FreeGaitActionServer::preemptCallback()
{
  ROS_INFO("StepAction is requested to preempt.");
  if (stepQueue_->empty()) return;
  if (stepQueue_->size() <= 1) return;
  stepQueue_->clearNextSteps();
  isPreempting_ = true;
}

void FreeGaitActionServer::publishFeedback()
{
  free_gait_msgs::ExecuteStepsFeedback feedback;
  if (stepQueue_->empty()) return;
  auto& step = stepQueue_->getCurrentStep();
  feedback.queue_size = stepQueue_->size();

//  if (step.checkStatus() == false) {
//    feedback.status = free_gait_msgs::ExecuteStepsFeedback::PROGRESS_PAUSED;
//    feedback.description = "Paused.";
//  } else {
//    switch (step.getState()) {
//      feedback.status = free_gait_msgs::ExecuteStepsFeedback::PROGRESS_EXECUTING;
//      case Step::State::PreStep:
//        feedback.description = "Pre step.";
//        break;
//      case Step::State::AtStep:
//        feedback.description = "At step.";
//        break;
//      case Step::State::PostStep:
//        feedback.description = "Post step.";
//        break;
//      default:
//        feedback.status = free_gait_msgs::ExecuteStepsFeedback::PROGRESS_UNKNOWN;
//        feedback.description = "Unknown.";
//        break;
//    }
//  }

  feedback.duration = ros::Duration(step.getTotalDuration());
  feedback.phase = step.getTotalPhase();
//  for (const auto& stepData : step.getSwingData())
//    feedback.swing_leg_names.push_back(stepData.first);

  server_.publishFeedback(feedback);
}

void FreeGaitActionServer::setSucceeded()
{
  ROS_INFO("StepAction succeeded.");
  free_gait_msgs::ExecuteStepsResult result;
  result.status = free_gait_msgs::ExecuteStepsResult::RESULT_REACHED;
  server_.setSucceeded(result, "Step action has been reached.");
}

void FreeGaitActionServer::setPreempted()
{
  ROS_INFO("StepAction preempted.");
  free_gait_msgs::ExecuteStepsResult result;
  result.status = free_gait_msgs::ExecuteStepsResult::RESULT_FAILED;
  server_.setPreempted(result, "Step action has been preempted.");
  isPreempting_ = false;
}

void FreeGaitActionServer::setAborted()
{
  ROS_INFO("StepAction aborted.");
  free_gait_msgs::ExecuteStepsResult result;
  result.status = free_gait_msgs::ExecuteStepsResult::RESULT_FAILED;
  server_.setAborted(result, "Step action has failed.");
}

} /* namespace */
