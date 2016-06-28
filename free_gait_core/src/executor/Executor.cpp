/*
 * Executor.cpp
 *
 *  Created on: Oct 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/executor/Executor.hpp"
#include "free_gait_core/leg_motion/JointTrajectory.hpp"

namespace free_gait {

Executor::Executor(std::shared_ptr<StepCompleter> completer,
                   std::shared_ptr<StepComputer> computer,
                   std::shared_ptr<AdapterBase> adapter,
                   std::shared_ptr<State> state)
    : completer_(completer),
      computer_(computer),
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
  computer_->initialize();
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
  if (!isInitialized_) return false;
  updateStateWithMeasurements();
  bool executionStatus = adapter_->isExecutionOk();

  if (executionStatus) {
    if (!state_->getRobotExecutionStatus()) std::cout << "Continuing with free gait execution." << std::endl;
    state_->setRobotExecutionStatus(true);
  } else {
    if (state_->getRobotExecutionStatus()) std::cout << "Robot status is not OK, not continuing with free gait execution." << std::endl;
    state_->setRobotExecutionStatus(false);
    return true;
  }

  // Copying result from computer when done.
  if (!queue_.empty() && queue_.getCurrentStep().needsComputation() && computer_->isDone()) {
     computer_->getStep(queue_.getCurrentStep());
     computer_->resetIsDone();
  }

  // Advance queue.
  if (!queue_.advance(dt)) return false;

  if (!adapter_->updateExtrasBefore(queue_, *state_)) return false;

  // For a new switch in step, do some work on step for the transition.
  while (queue_.hasSwitchedStep()) {
    auto& currentStep = queue_.getCurrentStep();
    if (!completer_->complete(*state_, queue_, currentStep)) {
      std::cerr << "Executor::advance: Could not complete step." << std::endl;
      return false;
    }
    if (currentStep.needsComputation() && !computer_->isBusy()) {
      computer_->setStep(currentStep);
      if (!computer_->compute()) {
        std::cerr << "Executor::advance: Could not compute step." << std::endl;
        return false;
      }
      if (computer_->isDone()) {
        computer_->getStep(queue_.getCurrentStep());
        computer_->resetIsDone();
      }
    }
    if (!queue_.advance(dt)) return false; // Advance again after completion.
  }

  if (queue_.hasStartedStep()) {
    std::cout << "Switched step to:" << std::endl;
    std::cout << queue_.getCurrentStep() << std::endl;
  }

  if (!writeIgnoreContact()) return false;
  if (!writeIgnoreForPoseAdaptation()) return false;
  if (!writeSupportLegs()) return false;
  if (!writeSurfaceNormals()) return false;
  if (!writeLegMotion()) return false;
  if (!writeTorsoMotion()) return false;
  if (!adapter_->updateExtrasAfter(queue_, *state_)) return false;
//  std::cout << *state_ << std::endl;

  return true;
}

void Executor::reset()
{
  queue_.clear();
  resetStateWithRobot();
  adapter_->resetExtrasWithRobot(queue_, *state_);
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

bool Executor::resetStateWithRobot()
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
  state_->setOrientationBaseToWorld(adapter_->getOrientationBaseToWorld());

  for (const auto& limb : adapter_->getLimbs()) {
    state_->setSupportLeg(limb, adapter_->isLegGrounded(limb));
    state_->setIgnoreContact(limb, !adapter_->isLegGrounded(limb));
  }

  state_->setRobotExecutionStatus(true);
  return true;
}

bool Executor::updateStateWithMeasurements()
{
  for (const auto& limb : adapter_->getLimbs()) {
    const auto& controlSetup = state_->getControlSetup(limb);
    if (!controlSetup.at(ControlLevel::Position)) {
      state_->setJointPositionsForLimb(limb, adapter_->getJointPositionsForLimb(limb));
    }
    if (!controlSetup.at(ControlLevel::Velocity)) {
      state_->setJointVelocitiesForLimb(limb, adapter_->getJointVelocitiesForLimb(limb));
    }
    if (!controlSetup.at(ControlLevel::Acceleration)) {
//      state_->setJointAccelerations(limb, adapter_->getJointAccelerations(limb));
    }
    if (!controlSetup.at(ControlLevel::Effort)) {
      state_->setJointEffortsForLimb(limb, adapter_->getJointEffortsForLimb(limb));
    }
  }

  const auto& controlSetup = state_->getControlSetup(BranchEnum::BASE);
  if (!controlSetup.at(ControlLevel::Position)) {
    state_->setPositionWorldToBaseInWorldFrame(adapter_->getPositionWorldToBaseInWorldFrame());
    state_->setOrientationBaseToWorld(adapter_->getOrientationBaseToWorld());
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
      } else {
        state_->removeSurfaceNormal(limb);
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
          const std::string& frameId = endEffectorMotion.getFrameId(ControlLevel::Position);
          if (!adapter_->frameIdExists(frameId)) {
            std::cerr << "Could not find frame '" << frameId << "' for free gait leg motion!" << std::endl;
            return false;
          }
          Position positionInBaseFrame = adapter_->transformPosition(frameId, "base", endEffectorMotion.evaluatePosition(time));
          JointPositionsLeg jointPositions;
          if (!adapter_->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(positionInBaseFrame, limb, jointPositions)) {
            std::cerr << "Failed to compute joint positions from end effector position for " <<limb << "." << std::endl;
            return false;
          }
          state_->setJointPositionsForLimb(limb, jointPositions);
        }
        break;
      }

      case LegMotionBase::TrajectoryType::Joints:
      {
        const auto& jointMotion = dynamic_cast<const JointMotionBase&>(legMotion);
        if (controlSetup[ControlLevel::Position]) state_->setJointPositionsForLimb(limb, jointMotion.evaluatePosition(time));
//        if (controlSetup[ControlLevel::Velocity]) state_->setJointVelocities(limb, jointMotion.evaluateVelocity(time));
//        if (controlSetup[ControlLevel::Acceleration]) state_->setJointAcceleration(limb, jointMotion.evaluateAcceleration(time));
        if (controlSetup[ControlLevel::Effort]) state_->setJointEffortsForLimb(limb, jointMotion.evaluateEffort(time));
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
  const auto& baseMotion = queue_.getCurrentStep().getBaseMotion();
  ControlSetup controlSetup = baseMotion.getControlSetup();
  state_->setControlSetup(BranchEnum::BASE, controlSetup);
  if (controlSetup[ControlLevel::Position]) {
    const std::string& frameId = baseMotion.getFrameId(ControlLevel::Position);
    if (!adapter_->frameIdExists(frameId)) {
      std::cerr << "Could not find frame '" << frameId << "' for free gait base motion!" << std::endl;
      return false;
    }
    Pose poseInWorldFrame = adapter_->transformPose(frameId, adapter_->getWorldFrameId(), baseMotion.evaluatePose(time));
    state_->setPositionWorldToBaseInWorldFrame(poseInWorldFrame.getPosition());
    state_->setOrientationBaseToWorld(poseInWorldFrame.getRotation());
  }
  if (controlSetup[ControlLevel::Velocity]) {
    // TODO Add frame handling.
    Twist twist = baseMotion.evaluateTwist(time);
    state_->setLinearVelocityBaseInWorldFrame(twist.getTranslationalVelocity());
    state_->setAngularVelocityBaseInBaseFrame(twist.getRotationalVelocity());
  }
  // TODO Set more states.
  return true;
}

} /* namespace free_gait */
