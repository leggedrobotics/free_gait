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
                   std::shared_ptr<AdapterBase> adapter,
                   std::shared_ptr<State> state)
    : completer_(completer),
      adapter_(adapter),
      state_(state),
      isInitialized_(false)
{
  // TODO Auto-generated constructor stub

}

Executor::~Executor()
{
  // TODO Auto-generated destructor stub
}

bool Executor::initialize()
{
  adapter_->initializeState(*state_);
  reset();
  return isInitialized_ = true;
}

bool Executor::isInitialized() const
{
  return isInitialized_;
}

bool Executor::advance(double dt)
{
  if (!isInitialized_) return false;
  bool hasSwitchedStep;
  if (!queue_.advance(dt, hasSwitchedStep)) return false;
  if (queue_.empty()) return true;

  while (hasSwitchedStep) {
    updateStateWithMeasurements();
    completer_->complete(*state_, queue_.getCurrentStep());

    if (hasSwitchedStep) {
      std::cout << "Switched step state to:" << std::endl;
      std::cout << queue_.getCurrentStep() << std::endl;
    }

    if (!queue_.advance(dt, hasSwitchedStep)) return false; // Advance again after completion.
    if (queue_.empty()) return true;
  }

  if (!writeIgnoreContact()) return false;
  if (!writeSupportLegs()) return false;
  if (!writeLegMotion()) return false;
  if (!writeTorsoMotion()) return false;
  if (!adapter_->updateExtras(queue_, *state_)) return false;


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

  return true;
}

void Executor::reset()
{
  queue_.clear();
  updateStateWithMeasurements();
}

const StepQueue& Executor::getQueue() const
{
  return queue_;
}

StepQueue& Executor::getQueue()
{
  return queue_;
}

const State& Executor::getState() const
{
  return *state_;
}

const AdapterBase& Executor::getAdapter() const
{
  return *adapter_;
}

bool Executor::updateStateWithMeasurements()
{
  // Update states for new step.
  if (!queue_.previousStepExists()) {
    // Update all states.
    adapter_->updateStateWithMeasurements(*state_);
  } else {
    // Update uncontrolled steps.
  }
}

bool Executor::writeIgnoreContact()
{
  const Step& step = queue_.getCurrentStep();
  for (const auto& limb : state_->getLimbs()) {
    if (step.hasLegMotion(limb)) {
      bool ignoreContact = step.getLegMotion(limb).isIgnoreContact();
      state_->setIgnoreContact(limb, ignoreContact);
    }
  }
  return true;
}

bool Executor::writeSupportLegs()
{
  const Step& step = queue_.getCurrentStep();
  for (const auto& limb : state_->getLimbs()) {
    if (step.hasLegMotion(limb) || state_->isIgnoreContact(limb)) {
      state_->setSupportLeg(limb, false);
    } else {
      state_->setSupportLeg(limb, true);
    }
  }
  return true;
}

bool Executor::writeLegMotion()
{
  const auto& step = queue_.getCurrentStep();
  if (!step.hasLegMotion()) return true;
  double time = queue_.getCurrentStep().getTime();
  for (const auto& limb : state_->getLimbs()) {
    if (!step.hasLegMotion(limb)) continue;
    auto const& legMotion = step.getLegMotion(limb);
    ControlSetup controlSetup = legMotion.getControlSetup();

    switch (legMotion.getTrajectoryType()) {

      case LegMotionBase::TrajectoryType::EndEffector:
      {
        const auto& endEffectorMotion = dynamic_cast<const EndEffectorMotionBase&>(legMotion);
        if (controlSetup[ControlLevel::Position]) {
          Position position = endEffectorMotion.evaluatePosition(time);
//          state_->setPositionWorldToBaseInWorldFrame(pose.getPosition());
        }
        break;
      }

      case LegMotionBase::TrajectoryType::Joints:
      {
        // TODO
        break;
      }

      default:
        throw std::runtime_error("Executor::writeLegMotion() could not write leg motion of this type.");
        break;
    }


  }
  return true;
}

bool Executor::writeTorsoMotion()
{
  if (!queue_.getCurrentStep().hasBaseMotion()) return true;
  double time = queue_.getCurrentStep().getTime();
  // TODO Add frame handling.
  const auto& baseMotion = queue_.getCurrentStep().getBaseMotion();
  ControlSetup controlSetup = baseMotion.getControlSetup();
  if (controlSetup[ControlLevel::Position]) {
    Pose pose = baseMotion.evaluatePose(time);
    state_->setPositionWorldToBaseInWorldFrame(pose.getPosition());
    state_->setOrientationWorldToBase(pose.getRotation());
  }
  if (controlSetup[ControlLevel::Velocity]) {
    Twist twist = baseMotion.evaluateTwist(time);
    state_->setLinearVelocityBaseInWorldFrame(twist.getTranslationalVelocity());
    state_->setAngularVelocityBaseInBaseFrame(twist.getRotationalVelocity());
  }
  // TODO Set more states.
  return true;
}

} /* namespace free_gait */
