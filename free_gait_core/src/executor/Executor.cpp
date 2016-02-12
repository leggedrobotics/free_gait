/*
 * Executor.cpp
 *
 *  Created on: Oct 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/executor/Executor.hpp"

#include <robotUtils/timers/ChronoTimer.hpp>

namespace free_gait {

Executor::Executor(std::shared_ptr<StepCompleter> completer,
                   std::shared_ptr<AdapterBase> adapter,
                   std::shared_ptr<State> state)
    : completer_(completer),
      adapter_(adapter),
      state_(state),
      isInitialized_(false)
{
}

Executor::~Executor()
{
}

bool Executor::initialize()
{
  Lock lock(mutex_);
  state_->initialize(adapter_->getLimbs(), adapter_->getBranches());
  reset();
  return isInitialized_ = true;
}

bool Executor::isInitialized() const
{
  return isInitialized_;
}

Executor::Mutex& Executor::getMutex()
{
  return mutex_;
}

bool Executor::advance(double dt)
{
  Lock lock(mutex_);

  if (!isInitialized_) return false;
  updateStateWithMeasurements();

  if (checkRobotStatus()) {
    if (!state_->getRobotExecutionStatus()) std::cout << "Continuing with free gait execution." << std::endl;
    state_->setRobotExecutionStatus(true);
  } else {
    if (state_->getRobotExecutionStatus()) std::cout << "Robot status is not OK, not continuing with free gait execution." << std::endl;
    state_->setRobotExecutionStatus(false);
    return true;
  }

  bool hasSwitchedStep;
  if (!queue_.advance(dt, hasSwitchedStep)) return false;

  while (hasSwitchedStep) {

    robotUtils::ChronoTimer timer;
    timer.pinTime();

    if (!completer_->complete(*state_, queue_, queue_.getCurrentStep())) {
      std::cerr << "Executor::advance: Could not complete step." << std::endl;
      return false;
    }

    std::cout << "Time to complete step: " << timer.getElapsedTimeMsec() << std::endl;

    if (hasSwitchedStep) {
      std::cout << "Switched step state to:" << std::endl;
      std::cout << queue_.getCurrentStep() << std::endl;
    }

    if (!queue_.advance(dt, hasSwitchedStep)) return false; // Advance again after completion.
  }

  if (!writeIgnoreContact()) return false;
  if (!writeIgnoreForPoseAdaptation()) return false;
  if (!writeSupportLegs()) return false;
  if (!writeSurfaceNormals()) return false;
  if (!writeLegMotion()) return false;
  if (!writeTorsoMotion()) return false;
  if (!adapter_->updateExtras(queue_, *state_)) return false;
//  std::cout << *state_ << std::endl;
  return true;
}

void Executor::reset()
{
  Lock lock(mutex_);
  queue_.clear();
  initializeStateWithRobot();
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

bool Executor::checkRobotStatus()
{
  for (const auto& limb : adapter_->getLimbs()) {
    if (state_->isSupportLeg(limb) && !adapter_->isLegGrounded(limb)) return false;
  }
  return true;
}

bool Executor::initializeStateWithRobot()
{
  for (const auto& limb : adapter_->getLimbs()) {
    state_->setSupportLeg(limb, adapter_->isLegGrounded(limb));
    state_->setIgnoreContact(limb, !adapter_->isLegGrounded(limb));
    state_->setIgnoreForPoseAdaptation(limb, !adapter_->isLegGrounded(limb));
    state_->removeSurfaceNormal(limb);
  }

  if (state_->getNumberOfSupportLegs() > 0) {
    state_->setControlSetup(BranchEnum::BASE, adapter_->getControlSetup(BranchEnum::BASE));
  } else {
    state_->setEmptyControlSetup(BranchEnum::BASE);
  }

  for (const auto& limb : adapter_->getLimbs()) {
    if (state_->isSupportLeg(limb)) {
      state_->setEmptyControlSetup(limb);
    } else {
      state_->setControlSetup(limb, adapter_->getControlSetup(limb));
    }
  }

  state_->setAllJointPositions(adapter_->getAllJointPositions());
  state_->setAllJointVelocities(adapter_->getAllJointVelocities());
//  state_->setAllJointAccelerations(adapter_->getAllJointAccelerations()); // TODO
  state_->setAllJointEfforts(adapter_->getAllJointEfforts());
  state_->setPositionWorldToBaseInWorldFrame(adapter_->getPositionWorldToBaseInWorldFrame());
  state_->setOrientationWorldToBase(adapter_->getOrientationWorldToBase());

  for (const auto& limb : adapter_->getLimbs()) {
    state_->setSupportLeg(limb, adapter_->isLegGrounded(limb));
    state_->setIgnoreContact(limb, !adapter_->isLegGrounded(limb));
  }

  state_->setRobotExecutionStatus(true);

  return true;
}

bool Executor::updateStateWithMeasurements()
{
  // Update states for new step.
//  if (!queue_.previousStepExists()) {
    // Update all states.

  for (const auto& limb : adapter_->getLimbs()) {
    const auto& controlSetup = state_->getControlSetup(limb);
    if (!controlSetup.at(ControlLevel::Position)) {
      state_->setJointPositions(limb, adapter_->getJointPositions(limb));
    }
    if (!controlSetup.at(ControlLevel::Velocity)) {
      state_->setJointVelocities(limb, adapter_->getJointVelocities(limb));
    }
    if (!controlSetup.at(ControlLevel::Acceleration)) {
//      state_->setJointAccelerations(limb, adapter_->getJointAccelerations(limb));
    }
    if (!controlSetup.at(ControlLevel::Effort)) {
      state_->setJointEfforts(limb, adapter_->getJointEfforts(limb));
    }
  }

  const auto& controlSetup = state_->getControlSetup(BranchEnum::BASE);
  if (!controlSetup.at(ControlLevel::Position)) {
    state_->setPositionWorldToBaseInWorldFrame(adapter_->getPositionWorldToBaseInWorldFrame());
    state_->setOrientationWorldToBase(adapter_->getOrientationWorldToBase());
  }

  state_->setAllJointVelocities(adapter_->getAllJointVelocities());
  // TODO Copy also acceleraitons and torques.
//    state.setLinearVelocityBaseInWorldFrame(torso_->getMeasuredState().getLinearVelocityBaseInBaseFrame());
//    state.setAngularVelocityBaseInBaseFrame(torso_->getMeasuredState().getAngularVelocityBaseInBaseFrame());
  return true;

}

bool Executor::writeIgnoreContact()
{
  if (!queue_.active()) return true;
  const Step& step = queue_.getCurrentStep();
  for (const auto& limb : adapter_->getLimbs()) {
    if (step.hasLegMotion(limb)) {
      bool ignoreContact = step.getLegMotion(limb).isIgnoreContact();
      state_->setIgnoreContact(limb, ignoreContact);
    }
  }
  return true;
}

bool Executor::writeIgnoreForPoseAdaptation()
{
  if (!queue_.active()) return true;
  const Step& step = queue_.getCurrentStep();
  for (const auto& limb : adapter_->getLimbs()) {
    if (step.hasLegMotion(limb)) {
      bool ignoreForPoseAdaptation = step.getLegMotion(limb).isIgnoreForPoseAdaptation();
      state_->setIgnoreForPoseAdaptation(limb, ignoreForPoseAdaptation);
    }
  }
  return true;
}

bool Executor::writeSupportLegs()
{
  if (!queue_.active()) {
    for (const auto& limb : adapter_->getLimbs()) {
      state_->setSupportLeg(limb, !state_->isIgnoreContact(limb));
    }
    return true;
  }

  const Step& step = queue_.getCurrentStep();
  for (const auto& limb : adapter_->getLimbs()) {
    if (step.hasLegMotion(limb) || state_->isIgnoreContact(limb)) {
      state_->setSupportLeg(limb, false);
    } else {
      state_->setSupportLeg(limb, true);
    }
  }
  return true;
}

bool Executor::writeSurfaceNormals()
{
  if (!queue_.active()) return true;
  const Step& step = queue_.getCurrentStep();
  for (const auto& limb : adapter_->getLimbs()) {
    if (step.hasLegMotion(limb)) {
      if (step.getLegMotion(limb).hasSurfaceNormal()) {
        state_->setSurfaceNormal(limb, step.getLegMotion(limb).getSurfaceNormal());
      }
    }
  }
  return true;
}

bool Executor::writeLegMotion()
{
  for (const auto& limb : adapter_->getLimbs()) {
    if (state_->isSupportLeg(limb)) state_->setEmptyControlSetup(limb);
  }
  if (!queue_.active()) return true;

  const auto& step = queue_.getCurrentStep();
  if (!step.hasLegMotion()) return true;

  double time = queue_.getCurrentStep().getTime();
  for (const auto& limb : adapter_->getLimbs()) {
    if (!step.hasLegMotion(limb)) continue;
    auto const& legMotion = step.getLegMotion(limb);
    ControlSetup controlSetup = legMotion.getControlSetup();
    state_->setControlSetup(limb, controlSetup);

    switch (legMotion.getTrajectoryType()) {

      case LegMotionBase::TrajectoryType::EndEffector:
      {
        const auto& endEffectorMotion = dynamic_cast<const EndEffectorMotionBase&>(legMotion);
        if (controlSetup[ControlLevel::Position]) {
          // TODO Add frame handling.
          Position positionInWorldFrame(endEffectorMotion.evaluatePosition(time));
          Position positionInBaseFrame(adapter_->getOrientationWorldToBase().rotate(positionInWorldFrame - adapter_->getPositionWorldToBaseInWorldFrame()));
          JointPositions jointPositions;
          adapter_->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(positionInBaseFrame, limb, jointPositions);
          state_->setJointPositions(limb, jointPositions);
        }
        break;
      }

      case LegMotionBase::TrajectoryType::Joints:
      {
        const auto& jointMotion = dynamic_cast<const JointMotionBase&>(legMotion);
        if (controlSetup[ControlLevel::Position]) state_->setJointPositions(limb, jointMotion.evaluatePosition(time));
//        if (controlSetup[ControlLevel::Velocity]) state_->setJointVelocities(limb, jointMotion.evaluateVelocity(time));
//        if (controlSetup[ControlLevel::Acceleration]) state_->setJointAcceleration(limb, jointMotion.evaluateAcceleration(time));
        if (controlSetup[ControlLevel::Effort]) state_->setJointEfforts(limb, jointMotion.evaluateEffort(time));
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
  if (state_->getNumberOfSupportLegs() == 0) state_->setEmptyControlSetup(BranchEnum::BASE);
  if (!queue_.active()) return true;

  if (!queue_.getCurrentStep().hasBaseMotion()) return true;
  double time = queue_.getCurrentStep().getTime();
  // TODO Add frame handling.
  const auto& baseMotion = queue_.getCurrentStep().getBaseMotion();
  ControlSetup controlSetup = baseMotion.getControlSetup();
  state_->setControlSetup(BranchEnum::BASE, controlSetup);
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
