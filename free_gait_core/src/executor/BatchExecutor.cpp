/*
 * BatchExecutor.cpp
 *
 *  Created on: Dec 20, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/executor/BatchExecutor.hpp"
#include "free_gait_core/executor/State.hpp"

namespace free_gait {

BatchExecutor::BatchExecutor(std::shared_ptr<free_gait::Executor> executor)
    : executor_(executor),
      timeStep_(0.001),
      isProcessing_(false),
      requestForCancelling_(false)
{
}

BatchExecutor::~BatchExecutor()
{
}

void BatchExecutor::addProcessingCallback(std::function<void(bool)> callback)
{
  callback_ = callback;
}

bool BatchExecutor::process(const free_gait::StepQueue& queue)
{
  if (isProcessing_) return false;
  isProcessing_ = true;
  queue_ = queue;
  std::thread thread(std::bind(&BatchExecutor::processInThread, this));
  thread.detach();
  return true;
}

bool BatchExecutor::isProcessing()
{
  return isProcessing_;
}

void BatchExecutor::cancelProcessing()
{
  requestForCancelling_ = true;
}

StateBatch BatchExecutor::getStateBatch() const
{
  if (isProcessing_) throw std::runtime_error("Batch executor error: Cannot access state during processing.");
  return stateBatch_;
}

void BatchExecutor::processInThread()
{
  stateBatch_.clear();
  free_gait::State state;
  for (size_t i = 0; i < 100; ++i) {
    state.setRandom();
    stateBatch_.addState((double) i/10.0, state);
  }
  isProcessing_ = false;
  callback_(true);
}

} /* namespace */
