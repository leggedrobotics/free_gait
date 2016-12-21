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
      playMode_(PlayMode::PAUSED),
      time_(0.0),
      stateRosPublisher_(nodeHandle, adapter),
      updateSleepDuration_(0.03) // 30 Hz update rate.
{
  std::shared_ptr<StepParameters> parameters(new StepParameters);
  std::shared_ptr<StepCompleter> completer(new StepCompleter(parameters, adapter_));
  std::shared_ptr<StepComputer> computer(new StepComputer());
  executorState_.reset(new State());
  std::shared_ptr<Executor> executor(new Executor(completer, computer, adapter_, executorState_));
  batchExecutor_.reset(new BatchExecutor(executor));
  batchExecutor_->addProcessingCallback(std::bind(&FreeGaitPreviewPlayback::processingCallback, this, std::placeholders::_1));
  updateThread_ = std::thread(std::bind(&FreeGaitPreviewPlayback::update, this));
  updateThread_.detach();
}

FreeGaitPreviewPlayback::~FreeGaitPreviewPlayback()
{
}

void FreeGaitPreviewPlayback::addNewGoalCallback(std::function<void()> callback)
{
  newGoalCallback_ = callback;
}
void FreeGaitPreviewPlayback::addReachedEndCallback(std::function<void()> callback)
{
  reachedEndCallback_ = callback;
}

bool FreeGaitPreviewPlayback::process(const free_gait::StepQueue& queue)
{
  return batchExecutor_->process(queue);
}

void FreeGaitPreviewPlayback::run()
{
  playMode_ = PlayMode::FORWARD;
}

void FreeGaitPreviewPlayback::stop()
{
  playMode_ = PlayMode::PAUSED;
}

void FreeGaitPreviewPlayback::clear()
{
  Lock lock(dataMutex_);
  playMode_ = PlayMode::PAUSED;
  time_.fromSec(0.0);
  stateBatch_.clear();
}

void FreeGaitPreviewPlayback::processingCallback(bool success)
{
  if (!success) return;
  Lock lock(dataMutex_);
  clear();
  stateBatch_ = batchExecutor_->getStateBatch();
  time_.fromSec(stateBatch_.getBeginTime());
  newGoalCallback_();
}

void FreeGaitPreviewPlayback::update()
{
  while (nodeHandle_.ok()) {
    switch (playMode_) {
      case PlayMode::FORWARD:
      {
        Lock lock(dataMutex_);
        time_ += ros::Duration(updateSleepDuration_.sec, updateSleepDuration_.nsec);
        if (time_ > ros::Time(stateBatch_.getEndTime())) {
          playMode_ = PlayMode::PAUSED;
          reachedEndCallback_();
        } else {
          publish(time_);
        }
      }
      case PlayMode::PAUSED:
      default:
        break;
    }
    updateSleepDuration_.sleep();
  }
}

void FreeGaitPreviewPlayback::publish(const ros::Time& time)
{
  const double timeInDouble = time.toSec();
  if (!stateBatch_.isValidTime(timeInDouble)) return;
  const State& state = stateBatch_.getState(timeInDouble);
  stateRosPublisher_.publish(state);
}

} /* namespace free_gait_rviz_plugin */
