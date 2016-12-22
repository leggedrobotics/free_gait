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
                                                 std::shared_ptr<free_gait::AdapterBase> adapter)
    : nodeHandle_(nodeHandle),
      adapter_(adapter),
      playMode_(PlayMode::ONHOLD),
      time_(0.0),
      stateBatchComputer_(adapter),
      stateRosPublisher_(nodeHandle, adapter)
{
  std::shared_ptr<StepParameters> parameters(new StepParameters);
  std::shared_ptr<StepCompleter> completer(new StepCompleter(parameters, adapter_));
  std::shared_ptr<StepComputer> computer(new StepComputer());
  executorState_.reset(new State());
  std::shared_ptr<Executor> executor(new Executor(completer, computer, adapter_, executorState_));
  executor->initialize();
  batchExecutor_.reset(new BatchExecutor(executor));
  batchExecutor_->addProcessingCallback(std::bind(&FreeGaitPreviewPlayback::processingCallback, this, std::placeholders::_1));
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
  playMode_ = PlayMode::FORWARD;
}

void FreeGaitPreviewPlayback::stop()
{
  playMode_ = PlayMode::STOPPED;
}

void FreeGaitPreviewPlayback::goToTime(const ros::Time& time)
{
  time_ = time;
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

void FreeGaitPreviewPlayback::update(double timeStep)
{
  switch (playMode_) {
    case PlayMode::FORWARD: {
      Lock lock(dataMutex_);
      time_ += ros::Duration(timeStep);
      if (time_ > ros::Time(stateBatch_.getEndTime())) {
        playMode_ = PlayMode::STOPPED;
        reachedEndCallback_();
      } else {
        publish(time_);
      }
      break;
    }
    case PlayMode::STOPPED: {
      publish(time_);
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
  if (!success) return;
  Lock lock(dataMutex_);
  clear();
  stateBatch_ = batchExecutor_->getStateBatch();
  stateBatchComputer_.computeEndEffectorTrajectories(stateBatch_);
  time_.fromSec(stateBatch_.getStartTime());
  newGoalCallback_();
}

void FreeGaitPreviewPlayback::publish(const ros::Time& time)
{
  const double timeInDouble = time.toSec();
  if (!stateBatch_.isValidTime(timeInDouble)) return;
  const State& state = stateBatch_.getState(timeInDouble);
  stateRosPublisher_.publish(state);
  stateChangedCallback_(time);
}

} /* namespace free_gait_rviz_plugin */
