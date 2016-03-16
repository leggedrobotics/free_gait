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

StepCompleter::StepCompleter(std::shared_ptr<StepParameters> parameters, std::shared_ptr<AdapterBase> adapter)
    : parameters_(parameters),
      adapter_(adapter)
{
}

StepCompleter::~StepCompleter()
{
}

bool StepCompleter::complete(const State& state, const StepQueue& queue, Step& step)
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

  return true;
}

bool StepCompleter::complete(const State& state, const Step& step, EndEffectorMotionBase& endEffectorMotion) const
{
  // Input.
//  ControlSetup controlSetupIn = state.getControlSetup(endEffectorMotion.getLimb());
//  bool positionIn = controlSetupIn.at(ControlLevel::Position);
//  bool velocityIn = controlSetupIn.at(ControlLevel::Velocity);
//  bool accelerationIn = controlSetupIn.at(ControlLevel::Acceleration);
//  bool effortIn = controlSetupIn.at(ControlLevel::Effort);

  // Output.
  ControlSetup controlSetupOut = endEffectorMotion.getControlSetup();
  bool positionOut = controlSetupOut.at(ControlLevel::Position);
  bool velocityOut = controlSetupOut.at(ControlLevel::Velocity);
  bool accelerationOut = controlSetupOut.at(ControlLevel::Acceleration);
  bool effortOut = controlSetupOut.at(ControlLevel::Effort);

  if (positionOut) {
    // TODO Check frame.
    Position startPositionInBaseFrame = adapter_->getPositionBaseToFootInBaseFrame(endEffectorMotion.getLimb(), state.getJointPositions(endEffectorMotion.getLimb()));
    Position startPositionInWorldFrame = adapter_->getPositionWorldToBaseInWorldFrame() + adapter_->getOrientationWorldToBase().inverseRotate(startPositionInBaseFrame);
    Position startPositionInDesiredFrame = startPositionInWorldFrame; // TODO
    endEffectorMotion.updateStartPosition(startPositionInDesiredFrame);
  }
  return endEffectorMotion.prepareComputation(state, step, *adapter_);
}

bool StepCompleter::complete(const State& state, const Step& step, JointMotionBase& jointMotion) const
{
  // Input.
  ControlSetup controlSetupIn = state.getControlSetup(jointMotion.getLimb());
  bool positionIn = controlSetupIn.at(ControlLevel::Position);
  bool velocityIn = controlSetupIn.at(ControlLevel::Velocity);
  bool accelerationIn = controlSetupIn.at(ControlLevel::Acceleration);
  bool effortIn = controlSetupIn.at(ControlLevel::Effort);

  // Output.
  ControlSetup controlSetupOut = jointMotion.getControlSetup();
  bool positionOut = controlSetupOut.at(ControlLevel::Position);
  bool velocityOut = controlSetupOut.at(ControlLevel::Velocity);
  bool accelerationOut = controlSetupOut.at(ControlLevel::Acceleration);
  bool effortOut = controlSetupOut.at(ControlLevel::Effort);

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
    if (positionOut) {
      JointPositions startPosition = state.getJointPositions(jointMotion.getLimb());
      jointMotion.updateStartPosition(startPosition);
    }
    if (velocityOut) {
      JointVelocities startVelocity = state.getJointVelocities(jointMotion.getLimb());
      jointMotion.updateStartVelocity(startVelocity);
    }
    if (accelerationOut) {
      JointAccelerations startAcceleration = state.getJointAccelerations(jointMotion.getLimb());
      jointMotion.updateStartAcceleration(startAcceleration);
    }
    if (effortOut) {
      JointEfforts startEffort = state.getJointEfforts(jointMotion.getLimb());
      jointMotion.updateStartEfforts(startEffort);
    }

  }
  return jointMotion.prepareComputation(state, step, *adapter_);
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
  return baseMotion.prepareComputation(state, step, queue, *adapter_);
}

void StepCompleter::setParameters(Footstep& footstep) const
{
  const auto& parameters = parameters_->footTargetParameters_;

  if (footstep.surfaceNormal_) {
    if (*(footstep.surfaceNormal_) == Vector::Zero())
      footstep.surfaceNormal_.reset(nullptr);
  }
  if (footstep.profileHeight_ == 0.0)
    footstep.profileHeight_ = parameters.profileHeight;
  if (footstep.profileType_.empty())
    footstep.profileType_ = parameters.profileType;
  if (footstep.averageVelocity_ == 0.0)
    footstep.averageVelocity_ = parameters.averageVelocity;

  footstep.liftOffVelocity_ = parameters.liftOffVelocity;
  footstep.touchdownVelocity_ = parameters.touchdownVelocity;
  footstep.minimumDuration_ = parameters.minimumDuration_;
}

void StepCompleter::setParameters(LegMode& legMode) const
{
  const auto& parameters = parameters_->legModeParameters_;

  if (legMode.surfaceNormal_) {
    if (*(legMode.surfaceNormal_) == Vector::Zero())
      legMode.surfaceNormal_.reset(nullptr);
  }
  if (legMode.duration_ == 0.0)
    legMode.duration_ = parameters.duration;
  if (legMode.frameId_.empty())
    legMode.frameId_ = parameters.frameId;
}

void StepCompleter::setParameters(BaseAuto& baseAuto) const
{
  const auto& parameters = parameters_->baseAutoParameters_;

  if (baseAuto.height_) {
    if (*(baseAuto.height_) == 0.0)
      baseAuto.height_.reset(nullptr);
  }
  if (baseAuto.averageLinearVelocity_ == 0.0)
    baseAuto.averageLinearVelocity_ = parameters.averageLinearVelocity;
  if (baseAuto.averageAngularVelocity_ == 0.0)
    baseAuto.averageAngularVelocity_ = parameters.averageAngularVelocity;
  if (baseAuto.supportMargin_ == 0.0)
    baseAuto.supportMargin_ = parameters.supportMargin;
  baseAuto.minimumDuration_ = parameters.minimumDuration_;

  baseAuto.nominalPlanarStanceInBaseFrame_.clear();
  baseAuto.nominalPlanarStanceInBaseFrame_ = parameters.nominalPlanarStanceInBaseFrame;
}

void StepCompleter::setParameters(BaseTrajectory& baseTrajectory) const
{
//  const auto& parameters = parameters_->baseAutoParameters_;
//
//  if (baseAuto.controllerType_.empty())
//    baseAuto.controllerType_ = parameters.controllerType;
}

} /* namespace */

