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
                                           std::shared_ptr<Executor> executor)
    : nodeHandle_(nodeHandle),
      name_(name),
      executor_(executor),
      rosConverter_(executor),
      server_(nodeHandle_, name_, false),
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
  Executor::SharedLock queueLock(executor_->getQueueMutex());
  bool stepQueueEmpty = executor_->getQueue().empty();
  queueLock.unlock();
  if (stepQueueEmpty) {
    // Succeeded.
    if (isPreempting_) {
      // Preempted.
      setPreempted();
    } else {
      setSucceeded();
    }
  } else {
    // Ongoing.
    queueLock.lock();
    if (executor_->getQueue().active()) publishFeedback();
    queueLock.unlock();
  }
}

void FreeGaitActionServer::shutdown()
{
  server_.shutdown();
}

void FreeGaitActionServer::goalCallback()
{
  ROS_INFO("Received goal for StepAction.");
//  if (server_.isActive()) server_.setRejected();

  const auto goal = server_.acceptNewGoal();
  Executor::UniqueLock queueLock(executor_->getQueueMutex());
  queueLock.unlock();
  for (auto& stepMessage : goal->steps) {
    Step step;
    rosConverter_.fromMessage(stepMessage, step);
    queueLock.lock();
    executor_->getQueue().add(step);
    queueLock.unlock();
  }
}

void FreeGaitActionServer::preemptCallback()
{
  ROS_INFO("StepAction is requested to preempt.");
  Executor::UniqueLock queueLock(executor_->getQueueMutex());
  if (executor_->getQueue().empty()) return;
  if (executor_->getQueue().size() <= 1) return;
  executor_->getQueue().clearNextSteps();
  isPreempting_ = true;
}

void FreeGaitActionServer::publishFeedback()
{
  free_gait_msgs::ExecuteStepsFeedback feedback;
  Executor::SharedLock queueLock(executor_->getQueueMutex());
  Executor::SharedLock adapterLock(executor_->getAdapterMutex());
  if (executor_->getQueue().empty()) return;
  // TODO Add feedback if executor multi-threading is not yet ready.
  const auto& step = executor_->getQueue().getCurrentStep();
  feedback.queue_size = executor_->getQueue().size();

  if (executor_->getAdapter().isExecutionOk() == false) {
    feedback.status = free_gait_msgs::ExecuteStepsFeedback::PROGRESS_PAUSED;
    feedback.description = "Paused.";
  } else {
      feedback.status = free_gait_msgs::ExecuteStepsFeedback::PROGRESS_EXECUTING;
      feedback.description = "Executing.";
//      default:
//        feedback.status = free_gait_msgs::ExecuteStepsFeedback::PROGRESS_UNKNOWN;
//        feedback.description = "Unknown.";
//        break;
  }

  feedback.duration = ros::Duration(step.getTotalDuration());
  feedback.phase = step.getTotalPhase();
  for (const auto& leg : executor_->getAdapter().getLimbs()) {
    if (step.hasLegMotion(leg)) {
      const std::string legName(executor_->getAdapter().getLimbStringFromLimbEnum(leg));
      feedback.swing_leg_names.push_back(legName);
    }
  }
  queueLock.unlock();
  adapterLock.unlock();
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
