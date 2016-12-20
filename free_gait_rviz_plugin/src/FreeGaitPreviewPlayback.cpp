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
      stateRosPublisher_(nodeHandle, adapter)
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

void FreeGaitPreviewPlayback::clear()
{
  playMode_ = PlayMode::PAUSED;
  time_.fromSec(0.0);
  stateBatch_.clear();
}

void FreeGaitPreviewPlayback::processingCallback(bool succcess)
{
  clear();
  stateBatch_ = batchExecutor_->getStateBatch();
  newGoalCallback_();
}

void FreeGaitPreviewPlayback::update()
{
  ros::WallDuration duration(0.03); // 30 Hz update rate.
  while (nodeHandle_.ok()) {
    switch (playMode_) {
      case PlayMode::FORWARD:
        time_ += ros::Duration(duration.sec, duration.nsec);
        publish(time_);
        break;
      default:
        break;
    }
    duration.sleep();
  }
}

void FreeGaitPreviewPlayback::publish(const ros::Time& time)
{
  const State& state = stateBatch_.getState(time.toSec());
  stateRosPublisher_.publish(state);
}

} /* namespace free_gait_rviz_plugin */
