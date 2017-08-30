/*
 * FreeGaitPreviewPlayback.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "free_gait_rviz_plugin/FreeGaitPreviewPlayback.hpp"

// STD
#include <iostream>

using namespace free_gait;

namespace free_gait_rviz_plugin {

FreeGaitPreviewPlayback::FreeGaitPreviewPlayback(ros::NodeHandle& nodeHandle,
                                                 free_gait::AdapterBase& adapter)
    : nodeHandle_(nodeHandle),
      playMode_(PlayMode::ONHOLD),
      time_(0.0),
      stateBatchComputer_(adapter),
      stateRosPublisher_(nodeHandle, adapter),
      speedFactor_(1.0)
{
  executorState_.reset(new State());

  parameters_.reset(new StepParameters);
  completer_.reset(new StepCompleter(*parameters_, adapter));
  computer_.reset(new StepComputer());

  executor_.reset(new Executor(*completer_, *computer_, adapter, *executorState_));
  executor_->initialize();

  batchExecutor_.reset(new BatchExecutor(*executor_));
  batchExecutor_->addProcessingCallback(
      std::bind(&FreeGaitPreviewPlayback::processingCallback, this, std::placeholders::_1));
}

FreeGaitPreviewPlayback::~FreeGaitPreviewPlayback()
{
}

void FreeGaitPreviewPlayback::addNewGoalCallback(std::function<void()> callback)
{
  newGoalCallback_ = callback;
}

void FreeGaitPreviewPlayback::addStateChangedCallback(std::function<void(const ros::Time&)> callback)
{
  stateChangedCallback_ = callback;
}

void FreeGaitPreviewPlayback::addReachedEndCallback(std::function<void()> callback)
{
  reachedEndCallback_ = callback;
}

bool FreeGaitPreviewPlayback::process(const std::vector<free_gait::Step>& steps)
{
  return batchExecutor_->process(steps);
}

void FreeGaitPreviewPlayback::run()
{
  ROS_DEBUG("Set mode to PlayMode::FORWARD.");
  playMode_ = PlayMode::FORWARD;
}

void FreeGaitPreviewPlayback::stop()
{
  ROS_DEBUG("Set mode to PlayMode::STOPPED.");
  playMode_ = PlayMode::STOPPED;
}

void FreeGaitPreviewPlayback::goToTime(const ros::Time& time)
{
  ROS_DEBUG_STREAM("Jumping to time " << time << ".");
  time_ = time;
  stop();
}

void FreeGaitPreviewPlayback::clear()
{
  Lock lock(dataMutex_);
  playMode_ = PlayMode::ONHOLD;
  time_.fromSec(0.0);
  stateBatch_.clear();
}

const free_gait::StateBatch& FreeGaitPreviewPlayback::getStateBatch() const
{
  return stateBatch_;
}

const ros::Time& FreeGaitPreviewPlayback::getTime() const
{
  return time_;
}

void FreeGaitPreviewPlayback::setSpeedFactor(const double speedFactor)
{
  speedFactor_ = speedFactor;
}

void FreeGaitPreviewPlayback::setRate(const double rate)
{
  batchExecutor_->setTimeStep(1.0/rate);
}

double FreeGaitPreviewPlayback::getRate() const
{
  return (1.0/batchExecutor_->getTimeStep());
}

void FreeGaitPreviewPlayback::setTfPrefix(const std::string tfPrefix)
{
  stateRosPublisher_.setTfPrefix(tfPrefix);
}

void FreeGaitPreviewPlayback::update(double timeStep)
{
  switch (playMode_) {
    case PlayMode::FORWARD: {
      Lock lock(dataMutex_);
      time_ += ros::Duration(speedFactor_ * timeStep);
      if (time_ > ros::Time(stateBatch_.getEndTime())) {
        stop();
        reachedEndCallback_();
      } else {
        publish(time_);
      }
      break;
    }
    case PlayMode::STOPPED: {
      publish(time_);
      ROS_DEBUG("Set mode to PlayMode::ONHOLD.");
      playMode_ = PlayMode::ONHOLD;
      break;
    }
    case PlayMode::ONHOLD:
    default:
      break;
  }
}

void FreeGaitPreviewPlayback::processingCallback(bool success)
{
  ROS_DEBUG("FreeGaitPreviewPlayback::processingCallback: Finished processing new goal, copying new data.");
  if (!success) return;
  Lock lock(dataMutex_);
  clear();
  stateBatch_ = batchExecutor_->getStateBatch(); // Deep copy.
  stateBatchComputer_.computeEndEffectorTrajectories(stateBatch_);
  stateBatchComputer_.computeEndEffectorTargetsAndSurfaceNormals(stateBatch_);
  stateBatchComputer_.computeStances(stateBatch_);
  stateBatchComputer_.computeStepIds(stateBatch_);
  time_.fromSec(stateBatch_.getStartTime());
  ROS_DEBUG_STREAM("Resetting time to " << time_ << ".");
  newGoalCallback_();
}

void FreeGaitPreviewPlayback::publish(const ros::Time& time)
{
  // TODO Increase speed by smarter locking.
  Lock lock(dataMutex_);
  const double timeInDouble = time.toSec();
  if (!stateBatch_.isValidTime(timeInDouble)) return;
  const State state = stateBatch_.getState(timeInDouble);
  stateRosPublisher_.publish(state);
  stateChangedCallback_(time);
}

} /* namespace free_gait_rviz_plugin */
