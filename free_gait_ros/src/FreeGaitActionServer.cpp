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
                                           Executor& executor, AdapterBase& adapter)
    : nodeHandle_(nodeHandle),
      name_(name),
      executor_(executor),
      adapter_(adapter),
      server_(nodeHandle_, name_, false),
      isPreempting_(false),
      isBlocked_(false),
      nStepsInCurrentGoal_(0)
{
}

FreeGaitActionServer::~FreeGaitActionServer()
{
}

void FreeGaitActionServer::initialize()
{
  server_.registerGoalCallback(boost::bind(&FreeGaitActionServer::goalCallback, this));
  server_.registerPreemptCallback(boost::bind(&FreeGaitActionServer::preemptCallback, this));
}

//void FreeGaitActionServer::setExecutor(std::shared_ptr<Executor> executor)
//{
//  Executor::Lock lock(executor_.getMutex());
//  executor_ = executor;
//}
//
//void FreeGaitActionServer::setAdapter(std::shared_ptr<AdapterBase> adapter)
//{
//  adapter_ = adapter;
//}

void FreeGaitActionServer::start()
{
  server_.start();
  ROS_INFO_STREAM("Started " << name_ << " action server.");
}

void FreeGaitActionServer::update()
{
  if (!server_.isActive() || isBlocked_) return;
  Executor::Lock lock(executor_.getMutex());
  bool stepQueueEmpty = executor_.getQueue().empty();
  lock.unlock();
  if (stepQueueEmpty) {
    if (nStepsInCurrentGoal_ == 0 ) {
      //Server is awaiting, action is idle
    }
    else if (isPreempting_) {
      // Preempted.
      setPreempted();
    } else {
      setSucceeded();
    }
  } else {
    // Ongoing.
    lock.lock();
    if (executor_.getQueue().active()) publishFeedback();
    lock.unlock();
  }
}

void FreeGaitActionServer::block()
{
  isBlocked_ = true;
}

void FreeGaitActionServer::unblock()
{
  isBlocked_ = false;
}

void FreeGaitActionServer::shutdown()
{
  ROS_INFO("Shutting down Free Gait Action Server.");
  server_.shutdown();
}

bool FreeGaitActionServer::isBlocked()
{
  return isBlocked_;
}

bool FreeGaitActionServer::isActive()
{
  return server_.isActive();
}

void FreeGaitActionServer::goalCallback()
{
  ROS_INFO("Received goal for StepAction.");
  if (isBlocked_) {
    ROS_WARN("StepAction server is blocked, goal will not be processed!");
    return;
  }

  const auto goal = server_.acceptNewGoal();

  // If goal's steps are empty, set server to wait
  if (goal->steps.empty()) {
    ROS_INFO("Received goal is void. Server will wait for next goal.");
  }

  std::vector<Step> steps;
  for (auto& stepMessage : goal->steps) {
    Step step;
    adapter_.fromMessage(stepMessage, step);
    steps.push_back(step);
  }
  Executor::Lock lock(executor_.getMutex());

  // Check if last step and first step of new goal are
  // pure `BaseAuto` commands: In this case, replace the
  // last one with the new one for smooth motion.
  if (executor_.getQueue().size() >= 2) {
    const auto& step = *executor_.getQueue().getQueue().end();
    if (!step.hasLegMotion() && step.hasBaseMotion()) {
      if (step.getBaseMotion().getType() == BaseMotionBase::Type::Auto) {
        executor_.getQueue().clearLastNSteps(1);
      }
    }
  }
  executor_.getQueue().add(steps);

  Executor::PreemptionType preemptionType;
  switch (goal->preempt) {
    case free_gait_msgs::ExecuteStepsGoal::PREEMPT_IMMEDIATE:
        preemptionType = Executor::PreemptionType::PREEMPT_IMMEDIATE;
      break;
    case free_gait_msgs::ExecuteStepsGoal::PREEMPT_STEP:
        preemptionType = Executor::PreemptionType::PREEMPT_STEP;
      break;
    case free_gait_msgs::ExecuteStepsGoal::PREEMPT_NO:
        preemptionType = Executor::PreemptionType::PREEMPT_NO;
      break;
    default:
      break;
  }
  executor_.setPreemptionType(preemptionType);
  nStepsInCurrentGoal_ = goal->steps.size();
  isPreempting_ = false;
  lock.unlock();
}

void FreeGaitActionServer::preemptCallback()
{
  ROS_INFO("StepAction is requested to preempt.");
  if (isBlocked_) {
    ROS_WARN("StepAction cannot be preempted, server is blocked!");
    return;
  }
  Executor::Lock lock(executor_.getMutex());
  executor_.stop();
  isPreempting_ = true;
}

void FreeGaitActionServer::publishFeedback()
{
  free_gait_msgs::ExecuteStepsFeedback feedback;
  Executor::Lock lock(executor_.getMutex());
  if (executor_.getQueue().empty()) return;
  feedback.step_id = executor_.getState().getStepId();
  feedback.queue_size = executor_.getQueue().size();
  feedback.number_of_steps_in_goal = nStepsInCurrentGoal_;
  feedback.step_number = feedback.number_of_steps_in_goal - feedback.queue_size + 1;

  if (executor_.getState().getRobotExecutionStatus() == false
      || executor_.getQueue().active() == false) {
    feedback.status = free_gait_msgs::ExecuteStepsFeedback::PROGRESS_PAUSED;
  } else {
      feedback.status = free_gait_msgs::ExecuteStepsFeedback::PROGRESS_EXECUTING;
//      default:
//        feedback.status = free_gait_msgs::ExecuteStepsFeedback::PROGRESS_UNKNOWN;
//        feedback.description = "Unknown.";
//        break;
  }

  feedback.description = executor_.getFeedbackDescription();
  executor_.clearFeedbackDescription();

  if (executor_.getQueue().active()) {
    const auto& step = executor_.getQueue().getCurrentStep();
    feedback.duration = ros::Duration(step.getTotalDuration());
    feedback.phase = step.getTotalPhase();
    for (const auto& legMotion : step.getLegMotions()) {
      const std::string legName(executor_.getAdapter().getLimbStringFromLimbEnum(legMotion.first));
      feedback.active_branches.push_back(legName);
    }
    if (step.hasBaseMotion()) {
      feedback.active_branches.push_back(executor_.getAdapter().getBaseString());
    }
  }

  lock.unlock();
  server_.publishFeedback(feedback);
}

void FreeGaitActionServer::setSucceeded()
{
  ROS_INFO("StepAction succeeded.");
  free_gait_msgs::ExecuteStepsResult result;
  server_.setSucceeded(result, "Step action has been reached.");
}

void FreeGaitActionServer::setPreempted()
{
  ROS_INFO("StepAction preempted.");
  free_gait_msgs::ExecuteStepsResult result;
  server_.setPreempted(result, "Step action has been preempted.");
  isPreempting_ = false;
}

void FreeGaitActionServer::setAborted()
{
  ROS_INFO("StepAction aborted.");
  free_gait_msgs::ExecuteStepsResult result;
  server_.setAborted(result, "Step action has failed (aborted).");
}

} /* namespace */
