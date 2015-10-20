/*
 * StepQueue.cpp
 *
 *  Created on: Jan 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/step/StepQueue.hpp"

// STD
#include <stdexcept>

// Roco
#include <roco/log/log_messages.hpp>

namespace free_gait {

StepQueue::StepQueue()
    : hasSwitchedStep_(false)
{
}

StepQueue::~StepQueue()
{
}

void StepQueue::add(const Step& step)
{
  queue_.push_back(step);
}

bool StepQueue::advance(double dt)
{
  if (queue_.empty()) return true;

  bool dequeue = true;
  hasSwitchedStep_ = false;
  while (dequeue) {
    if (!queue_.front().advance(dt)) {
      queue_.pop_front();
      if (queue_.empty()) return true;
      hasSwitchedStep_ = true;
    } else {
      dequeue = false;
    }
  }

  if (hasSwitchedStep() || getCurrentStep().hasSwitchedState()) {
    ROCO_DEBUG_STREAM("Switched step state to:");
    ROCO_DEBUG_STREAM(getCurrentStep());
  }

  return true;
}

bool StepQueue::empty() const
{
  return queue_.empty();
}

void StepQueue::clearNextSteps()
{
  if (empty()) return;
  queue_.erase(queue_.begin() + 1, queue_.end());
}

void StepQueue::clear()
{
  queue_.clear();
}

Step& StepQueue::getCurrentStep()
{
  if (empty()) throw std::out_of_range("No footsteps in queue!");
  return queue_.front();
}

Step& StepQueue::getNextStep()
{
  if (size() <= 1) throw std::out_of_range("No next footstep in queue!");
  auto itertator = queue_.begin() + 1;
  return *itertator;
}

std::deque<Step>::size_type StepQueue::size() const
{
  return queue_.size();
}

bool StepQueue::hasSwitchedStep() const
{
  return hasSwitchedStep_;
}

} /* namespace */
