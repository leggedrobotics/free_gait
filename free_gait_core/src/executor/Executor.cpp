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
  state_->initialize(adapter_->getLimbs(), adapter_->getBranches());
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
  updateStateWithMeasurements();
  SharedLock adapterLock(adapterMutex_);
  bool executionStatus = adapter_->isExecutionOk();
  adapterLock.unlock();

  UniqueLock stateLock(stateMutex_);
  if (executionStatus) {
    if (!state_->getRobotExecutionStatus()) std::cout << "Continuing with free gait execution." << std::endl;
    state_->setRobotExecutionStatus(true);
  } else {
    if (state_->getRobotExecutionStatus()) std::cout << "Robot status is not OK, not continuing with free gait execution." << std::endl;
    state_->setRobotExecutionStatus(false);
    return true;
  }
  stateLock.unlock();

  bool hasSwitchedStep;
  UniqueLock queueUniqueLock(queueMutex_);
  if (!queue_.advance(dt, hasSwitchedStep)) return false;
  queueUniqueLock.unlock();

  while (hasSwitchedStep) {

    queueUniqueLock.lock();
    if (!queue_.getCurrentStep().requiresMultiThreading()) {
      if (!completeCurrentStep()) return false;
      if (!queue_.advance(dt, hasSwitchedStep)) return false; // Advance again after completion.
      queueUniqueLock.unlock();
    } else {
      std::thread thread(&Executor::completeCurrentStep, this, true);
      thread.detach();
      queueUniqueLock.unlock();
      hasSwitchedStep = false;
    }
  }

  stateLock.lock();
  SharedLock queueSharedLock(queueMutex_);
  if (!writeIgnoreContact()) return false;
  if (!writeIgnoreForPoseAdaptation()) return false;
  if (!writeSupportLegs()) return false;
  if (!writeSurfaceNormals()) return false;
  if (!writeLegMotion()) return false;
  if (!writeTorsoMotion()) return false;
  adapterLock.lock();
  if (!adapter_->updateExtras(queue_, *state_)) return false;
  adapterLock.unlock();
  queueSharedLock.unlock();
  stateLock.unlock();
//  std::cout << *state_ << std::endl;

  return true;
}

bool Executor::completeCurrentStep(bool multiThreaded)
{
  std::cout << "START completeCurrentStep" << std::endl;
  robotUtils::HighResolutionClockTimer timer("Executor::completeCurrentStep");
  timer.pinTime();

  bool completionSuccessful;
  if (multiThreaded) {
    timer.pinTime("Copy state, queue, and step");
    SharedLock stateLock(stateMutex_);
    const State state(*state_);
    stateLock.unlock();
    SharedLock queueSharedLock(queueMutex_);
    Step step(queue_.getCurrentStep());
    queueSharedLock.unlock();
    timer.splitTime("Copy state, queue, and step");

    queueSharedLock.lock();
    SharedLock adapterLock(adapterMutex_);
    completionSuccessful = completer_->complete(state, queue_, step);
    adapterLock.unlock();
    queueSharedLock.unlock();

    UniqueLock queueUniqueLock(queueMutex_);
    queue_.replaceCurrentStep(step);
    queueUniqueLock.unlock();
  } else {
    completionSuccessful = completer_->complete(*state_, queue_, queue_.getCurrentStep());
  }

  if (!completionSuccessful) {
    std::cerr << "Executor::advance: Could not complete step." << std::endl;
    return false;
  }

  timer.splitTime();
  std::cout << timer << std::endl;
  std::cout << "Switched step to:" << std::endl;
  if (multiThreaded) {
    SharedLock queueLock(queueMutex_);
    std::cout << queue_.getCurrentStep() << std::endl;
    queueLock.unlock();
  } else {
    std::cout << queue_.getCurrentStep() << std::endl;
  }
  std::cout << "END completeCurrentStep" << std::endl;
  return true;
}

void Executor::reset()
{
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

Executor::Mutex& Executor::getQueueMutex()
{
  return queueMutex_;
}

const State& Executor::getState() const
{
  return *state_;
}

Executor::Mutex& Executor::getStateMutex()
{
  return stateMutex_;
}

const AdapterBase& Executor::getAdapter() const
{
  return *adapter_;
}

Executor::Mutex& Executor::getAdapterMutex()
{
  return adapterMutex_;
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
  SharedLock adapterLock(adapterMutex_);
  UniqueLock stateLock(stateMutex_);
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
          // TODO Add frame handling.
          Position positionInWorldFrame(endEffectorMotion.evaluatePosition(time));
          // TODO adapter_->getOrientationWorldToBase() is nicer when the robot slips, but leads to problems at lift-off.
          Position positionInBaseFrame(adapter_->getOrientationWorldToBase().rotate(positionInWorldFrame - adapter_->getPositionWorldToBaseInWorldFrame()));
//          Position positionInBaseFrame(state_->getOrientationWorldToBase().rotate(positionInWorldFrame - adapter_->getPositionWorldToBaseInWorldFrame()));
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
