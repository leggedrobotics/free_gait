/*
 * Executor.cpp
 *
 *  Created on: Oct 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/executor/Executor.hpp"

namespace free_gait {

Executor::Executor(std::shared_ptr<StepCompleter> completer,
                   std::shared_ptr<ExecutorAdapterBase> adapter,
                   std::shared_ptr<ExecutorState> state)
    : completer_(completer),
      adapter_(adapter),
      state_(state)
{
  // TODO Auto-generated constructor stub

}

Executor::~Executor()
{
  // TODO Auto-generated destructor stub
}

bool Executor::advance(double dt)
{
  if (!queue_.advance(dt)) return false;
  if (queue_.empty()) return true;

  if (queue_.hasSwitchedStep() && !queue_.getCurrentStep().isComplete()) {
    updateStateWithMeasurements();
  }

  adapter_->updateExtras(queue_, *state_);

//  if (queue_.hasSwitchedStep() && !queue_.getCurrentStep()->isComplete()) {
//    completer_->complete(*queue_.getCurrentStep());
//    // Update actuation type and robot state.
//    if (!queue_.getPreviousStep()) {
//      updateWithMeasuredBasePose();
//      updateWithMeasuredBaseTwist();
//      for (const auto& legControlSetup : legControlSetups_) {
//        updateWithMeasuredJointPositions(legControlSetup.first);
//        updateWithMeasuredJointVelocities(legControlSetup.first);
//        updateWithMeasuredJointEfforts(legControlSetup.first);
//        updateWithActualSupportLeg(legControlSetup.first);
//      }
//    } else {
//      auto& previousBaseSetup = queue_.getPreviousStep()->getCurrentBaseMotion().getControlSetup();
//      if (!previousBaseSetup.at(ControlLevel::Position)) updateWithMeasuredBasePose();
//      if (!previousBaseSetup.at(ControlLevel::Velocity)) updateWithMeasuredBaseTwist();
//
//      for (const auto& legControlSetup : legControlSetups_) {
//        // TODO: Special treatment for non-specified legs.
//        auto& previousLegSetup = queue_.getPreviousStep()->getLegMotions().at(legControlSetup.first).getControlSetup();
//        if (!previousBaseSetup.at(ControlLevel::Position)) updateWithMeasuredJointPositions(legControlSetup.first);
//        if (!previousBaseSetup.at(ControlLevel::Velocity)) updateWithMeasuredJointVelocities(legControlSetup.first);
//        // TODO etc.!!!
//      }
//    }
//
//    baseControlSetup_ = queue_.getCurrentStep()->getCurrentBaseMotion().getControlSetup();
//    for (const auto& legMotion : queue_.getCurrentStep()->getLegMotions()) {
//      legControlSetups_.at(legMotion.first) = legMotion.second.getControlSetup();
//    }
//  }
//
//  auto step = queue_.getCurrentStep();
//  // TODO Do this with all frame handling.
//  if (baseControlSetup_.at(ControlLevel::Position)) basePose_ = step->getCurrentBaseMotion().evaluatePose(step->getCurrentStateTime());
//  return true;
}

void Executor::reset()
{
  queue_.clear();
}

const StepQueue& Executor::getQueue() const
{
  return queue_;
}

const ExecutorState& Executor::getState() const
{
  return *state_;
}

bool Executor::updateStateWithMeasurements()
{

}

bool Executor::updateIgnoreContact()
{
  const Step& step = queue_.getCurrentStep();
  for (const auto& leg : state_->getIsIgnoreContact()) {
    if (step.hasLegMotion(leg.first)) {
      bool ignoreContact = step.getLegMotion(leg.first).isIgnoreContact();
      state_->setIgnoreContact(leg.first, ignoreContact);
    }
  }
  return true;
}

bool Executor::updateSupportLegs()
{
  const Step& step = queue_.getCurrentStep();
  for (const auto& leg : state_->getIsSupportLegs()) {
    if (step.hasLegMotion(leg.first) || state_->isIgnoreContact(leg.first)) {
      state_->setSupportLeg(leg.first, false);
    } else {
      state_->setSupportLeg(leg.first, true);
    }
  }
  return true;
}


} /* namespace free_gait */
