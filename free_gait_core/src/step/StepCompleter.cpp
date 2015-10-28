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

bool StepCompleter::complete(const State& state, Step& step) const
{
//  for (auto& legMotion : step.legMotions_) {
//    switch (legMotion.second.getType()) {
//      case LegMotionBase::Type::Footstep:
//        if (!complete(dynamic_cast<Footstep&>(legMotion.second))) return false;
//        break;
//      default:
//        break;
//    }
//
//  }
//


  if (step.baseMotion_) {
    switch (step.baseMotion_->getType()) {
      case BaseMotionBase::Type::Auto:
        setParameters(dynamic_cast<BaseAuto&>(*step.baseMotion_));
        break;
      default:
        break;
    }
    if (!complete(state, step, *(step.baseMotion_))) return false;
  }

  step.isComplete_ = true;
  return step.update();
}

bool StepCompleter::complete(const State& state, const Step& step, BaseMotionBase& baseMotion) const
{
  if (baseMotion.getControlSetup().at(ControlLevel::Position)) {
    // TODO Check frame.
    Pose pose(state.getPositionWorldToBaseInWorldFrame(), state.getOrientationWorldToBase());
    baseMotion.updateStartPose(pose);
  }
  if (baseMotion.getControlSetup().at(ControlLevel::Velocity)) {
    // TODO
  }
  baseMotion.compute(state, step, *adapter_);
  return true;
}

void StepCompleter::setParameters(Footstep& footstep) const
{
  if (footstep.getSurfaceNormal() == Vector::Zero())
    footstep.setSurfaceNormal(footTargetParameters_.surfaceNormal);
  if (footstep.getProfileHeight() == 0.0)
    footstep.setProfileHeight(footTargetParameters_.profileHeight);
  if (footstep.getProfileType().empty())
    footstep.setProfileType(footTargetParameters_.profileType);
  if (footstep.getAverageVelocity() == 0.0)
    footstep.setAverageVelocity(footTargetParameters_.averageVelocity);
}

void StepCompleter::setParameters(BaseAuto& baseAuto) const
{
  if (baseAuto.height_ == 0.0)
    baseAuto.height_ = baseAutoParameters_.height;
  if (baseAuto.averageLinearVelocity_ == 0.0)
    baseAuto.averageLinearVelocity_ = baseAutoParameters_.averageLinearVelocity;
  if (baseAuto.averageAngularVelocity_ == 0.0)
    baseAuto.averageAngularVelocity_ = baseAutoParameters_.averageAngularVelocity;
  if (baseAuto.supportMargin_ == 0.0)
    baseAuto.supportMargin_ = baseAutoParameters_.supportMargin;

  baseAuto.nominalPlanarStanceInBaseFrame_.clear();
  baseAuto.nominalPlanarStanceInBaseFrame_ = baseAutoParameters_.nominalPlanarStanceInBaseFrame;
}

} /* namespace */

