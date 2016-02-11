/*
 * StepCompleter.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/step/StepCompleter.hpp"
#include "free_gait_core/leg_motion/leg_motion.hpp"

namespace free_gait {

StepCompleter::StepCompleter(std::shared_ptr<free_gait::AdapterBase> adapter)
    : adapter_(adapter)
{
}

StepCompleter::~StepCompleter()
{
}

bool StepCompleter::complete(const State& state, const StepQueue& queue, Step& step) const
{
  for (auto& legMotion : step.legMotions_) {
    switch ((legMotion.second)->getType()) {
      case LegMotionBase::Type::Footstep:
        setParameters(dynamic_cast<Footstep&>(*legMotion.second));
        break;
      case LegMotionBase::Type::LegMode:
        setParameters(dynamic_cast<LegMode&>(*legMotion.second));
        break;
      default:
        break;
    }
    switch (legMotion.second->getTrajectoryType()) {
      case LegMotionBase::TrajectoryType::EndEffector:
        if (!complete(state, step, dynamic_cast<EndEffectorMotionBase&>(*legMotion.second))) return false;
        break;
      case LegMotionBase::TrajectoryType::Joints:
        if (!complete(state, step, dynamic_cast<JointMotionBase&>(*legMotion.second))) return false;
        break;
      default:
        throw std::runtime_error("StepCompleter::complete() could not complete leg motion of this type.");
        break;
    }
  }

  if (step.baseMotion_) {
    switch (step.baseMotion_->getType()) {
      case BaseMotionBase::Type::Auto:
        setParameters(dynamic_cast<BaseAuto&>(*step.baseMotion_));
        break;
      case BaseMotionBase::Type::Trajectory:
        setParameters(dynamic_cast<BaseTrajectory&>(*step.baseMotion_));
        break;
      default:
        break;
    }
    if (!complete(state, step, queue, *(step.baseMotion_))) return false;
  }

  step.isComplete_ = true;
  return step.update();
}

bool StepCompleter::complete(const State& state, const Step& step, EndEffectorMotionBase& endEffectorMotion) const
{
  if (endEffectorMotion.getControlSetup().at(ControlLevel::Position)) {
    // TODO Check frame.
    Position startPositionInBaseFrame = adapter_->getPositionBaseToFootInBaseFrame(endEffectorMotion.getLimb(), state.getJointPositions(endEffectorMotion.getLimb()));
    Position startPositionInWorldFrame = adapter_->getPositionWorldToBaseInWorldFrame() + adapter_->getOrientationWorldToBase().inverseRotate(startPositionInBaseFrame);
    Position startPositionInDesiredFrame = startPositionInWorldFrame; // TODO
    endEffectorMotion.updateStartPosition(startPositionInDesiredFrame);
  }
  return endEffectorMotion.compute(state, step, *adapter_);
}

bool StepCompleter::complete(const State& state, const Step& step, JointMotionBase& jointMotion) const
{
  // Input.
  bool positionIn = state.getControlSetup(jointMotion.getLimb()).at(ControlLevel::Position);
  bool velocityIn = state.getControlSetup(jointMotion.getLimb()).at(ControlLevel::Velocity);
  bool accelerationIn = state.getControlSetup(jointMotion.getLimb()).at(ControlLevel::Acceleration);
  bool effortIn = state.getControlSetup(jointMotion.getLimb()).at(ControlLevel::Effort);

  // Output.
  bool positionOut = jointMotion.getControlSetup().at(free_gait::ControlLevel::Position);
  bool velocityOut = jointMotion.getControlSetup().at(free_gait::ControlLevel::Velocity);
  bool accelerationOut = jointMotion.getControlSetup().at(free_gait::ControlLevel::Acceleration);
  bool effortOut = jointMotion.getControlSetup().at(free_gait::ControlLevel::Effort);

  // Check for special mode transitions.
  if (positionIn && !effortIn && positionOut && effortOut) {
    JointPositions startPosition = state.getJointPositions(jointMotion.getLimb());
    jointMotion.updateStartPosition(startPosition);
    JointEfforts startEffort = state.getJointEfforts(jointMotion.getLimb());
    startEffort.setZero();
    jointMotion.updateStartEfforts(startEffort);
  } else if (positionIn && effortIn && !positionOut && effortOut) {
    JointEfforts startEffort = adapter_->getJointEfforts(jointMotion.getLimb());
    jointMotion.updateStartEfforts(startEffort);
  } //else if (positionIn && effortIn && positionOut && !effortOut) {
    // TODO: Decrease torque to zero in special step.
  //} else if (!positionIn && effortIn && positionOut && !effortOut) {
    // TODO: Decrease compression on leg in special step.
  // }
  else {

    // Standard transitions.
    if (jointMotion.getControlSetup().at(ControlLevel::Position)) {
      JointPositions startPosition = state.getJointPositions(jointMotion.getLimb());
      jointMotion.updateStartPosition(startPosition);
    }
    if (jointMotion.getControlSetup().at(ControlLevel::Velocity)) {
      JointVelocities startVelocity = state.getJointVelocities(jointMotion.getLimb());
      jointMotion.updateStartVelocity(startVelocity);
    }
    if (jointMotion.getControlSetup().at(ControlLevel::Acceleration)) {
      JointAccelerations startAcceleration = state.getJointAccelerations(jointMotion.getLimb());
      jointMotion.updateStartAcceleration(startAcceleration);
    }
    if (jointMotion.getControlSetup().at(ControlLevel::Effort)) {
      JointEfforts startEffort = state.getJointEfforts(jointMotion.getLimb());
      jointMotion.updateStartEfforts(startEffort);
    }

  }
  return jointMotion.compute(state, step, *adapter_);
}

bool StepCompleter::complete(const State& state, const Step& step, const StepQueue& queue, BaseMotionBase& baseMotion) const
{
  if (baseMotion.getControlSetup().at(ControlLevel::Position)) {
    // TODO Check frame.
    Pose pose(state.getPositionWorldToBaseInWorldFrame(), state.getOrientationWorldToBase());
    baseMotion.updateStartPose(pose);
  }
  if (baseMotion.getControlSetup().at(ControlLevel::Velocity)) {
    // TODO
  }
  return baseMotion.compute(state, step, queue, *adapter_);
}

void StepCompleter::setParameters(Footstep& footstep) const
{
  if (footstep.surfaceNormal_) {
    if (*(footstep.surfaceNormal_) == Vector::Zero())
      footstep.surfaceNormal_.reset(nullptr);
  }
  if (footstep.profileHeight_ == 0.0)
    footstep.profileHeight_ = footTargetParameters_.profileHeight;
  if (footstep.profileType_.empty())
    footstep.profileType_ = footTargetParameters_.profileType;
  if (footstep.averageVelocity_ == 0.0)
    footstep.averageVelocity_ = footTargetParameters_.averageVelocity;
}

void StepCompleter::setParameters(LegMode& legMode) const
{
  if (legMode.surfaceNormal_) {
    if (*(legMode.surfaceNormal_) == Vector::Zero())
      legMode.surfaceNormal_.reset(nullptr);
  }
  if (legMode.duration_ == 0.0)
    legMode.duration_ = legModeParameters_.duration;
  if (legMode.frameId_.empty())
    legMode.frameId_ = legModeParameters_.frameId;
}

void StepCompleter::setParameters(BaseAuto& baseAuto) const
{
  if (baseAuto.height_) {
    if (*(baseAuto.height_) == 0.0)
      baseAuto.height_.reset(nullptr);
  }
  if (baseAuto.averageLinearVelocity_ == 0.0)
    baseAuto.averageLinearVelocity_ = baseAutoParameters_.averageLinearVelocity;
  if (baseAuto.averageAngularVelocity_ == 0.0)
    baseAuto.averageAngularVelocity_ = baseAutoParameters_.averageAngularVelocity;
  if (baseAuto.supportMargin_ == 0.0)
    baseAuto.supportMargin_ = baseAutoParameters_.supportMargin;

  baseAuto.nominalPlanarStanceInBaseFrame_.clear();
  baseAuto.nominalPlanarStanceInBaseFrame_ = baseAutoParameters_.nominalPlanarStanceInBaseFrame;
}

void StepCompleter::setParameters(BaseTrajectory& baseTrajectory) const
{
}

} /* namespace */

