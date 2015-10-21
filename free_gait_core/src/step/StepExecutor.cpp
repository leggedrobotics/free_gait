/*
 * StepExecutor.cpp
 *
 *  Created on: Oct 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/step/StepExecutor.hpp>

namespace free_gait {

StepExecutor::StepExecutor(std::shared_ptr<StepCompleter> completer)
    : completer_(completer)
{
  // TODO Auto-generated constructor stub

}

StepExecutor::~StepExecutor()
{
  // TODO Auto-generated destructor stub
}

bool StepExecutor::advance(double dt)
{
  if (!queue_.advance(dt)) return false;
  if (queue_.empty()) return true;

  // TODO How about first step?
  // TODO Instead, do this for each state.
  if (queue_.hasSwitchedStep() && !queue_.getCurrentStep()->isComplete()) {
    completer_->complete(*queue_.getCurrentStep());
    // Update actuation type and robot state.
    if (!queue_.getPreviousStep()) {
      updateWithMeasuredBasePose();
      updateWithMeasuredBaseTwist();
      for (const auto& legControlSetup : legControlSetups_) {
        updateWithMeasuredJointPositions(legControlSetup.first);
        updateWithMeasuredJointVelocities(legControlSetup.first);
        updateWithMeasuredJointEfforts(legControlSetup.first);
        updateWithActualSupportLeg(legControlSetup.first);
      }
    } else {
      auto& previousBaseSetup = queue_.getPreviousStep()->getCurrentBaseMotion().getControlSetup();
      if (!previousBaseSetup.at(ControlLevel::Position)) updateWithMeasuredBasePose();
      if (!previousBaseSetup.at(ControlLevel::Velocity)) updateWithMeasuredBaseTwist();

      for (const auto& legControlSetup : legControlSetups_) {
        // TODO: Special treatment for non-specified legs.
        auto& previousLegSetup = queue_.getPreviousStep()->getLegMotions().at(legControlSetup.first).getControlSetup();
        if (!previousBaseSetup.at(ControlLevel::Position)) updateWithMeasuredJointPositions(legControlSetup.first);
        if (!previousBaseSetup.at(ControlLevel::Velocity)) updateWithMeasuredJointVelocities(legControlSetup.first);
        // TODO etc.!!!
      }
    }

    baseControlSetup_ = queue_.getCurrentStep()->getCurrentBaseMotion().getControlSetup();
    for (const auto& legMotion : queue_.getCurrentStep()->getLegMotions()) {
      legControlSetups_.at(legMotion.first) = legMotion.second.getControlSetup();
    }
  }

  auto step = queue_.getCurrentStep();
  // TODO Do this with all frame handling.
  if (baseControlSetup_.at(ControlLevel::Position)) basePose_ = step->getCurrentBaseMotion().evaluatePose(step->getCurrentStateTime());
  return true;
}

StepQueue& StepExecutor::getQueue()
{
  return queue_;
}

const ControlSetup& StepExecutor::getBaseControlSetup() const
{
  return baseControlSetup_;
}

const Pose& StepExecutor::getBasePose()
{
  return basePose_;
}

const ControlSetup& StepExecutor::getLegControlSetup(LimbEnum limb) const
{
  return legControlSetups_.at(limb);
}

const JointPositions& StepExecutor::getJointPositions(LimbEnum limb) const
{
  return jointPositions_.at(limb);
}

const JointVelocities& StepExecutor::getJointVelocities(LimbEnum limb) const
{
  return jointVelocities_.at(limb);
}

const JointEfforts& StepExecutor::getJointEfforts(LimbEnum limb) const
{
  return jointEfforts_.at(limb);
}

bool StepExecutor::isSupportLeg(LimbEnum limb) const
{
  return isSupportLegs_.at(limb);
}

bool StepExecutor::isIgnoreContact(LimbEnum limb) const
{
  return ignoreContact_.at(limb);
}

} /* namespace free_gait */
