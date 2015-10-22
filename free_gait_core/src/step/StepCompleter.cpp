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

StepCompleter::StepCompleter()
{
}

StepCompleter::~StepCompleter()
{
}

bool StepCompleter::complete(Step& step) const
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
//  for (auto& baseMotion : step.baseMotions_) {
//    switch (baseMotion.second.getType()) {
//      case BaseMotionBase::Type::Auto:
//        if (!complete(dynamic_cast<BaseAuto&>(baseMotion.second))) return false;
//        break;
//      default:
//        break;
//    }
//  }

  return step.isComplete_ = true;
}

bool StepCompleter::complete(Footstep& footstep) const
{
  if (footstep.getSurfaceNormal() == Vector::Zero())
    footstep.setSurfaceNormal(footTargetParameters_.surfaceNormal);
  if (footstep.getProfileHeight() == 0.0)
    footstep.setProfileHeight(footTargetParameters_.profileHeight);
  if (footstep.getProfileType().empty())
    footstep.setProfileType(footTargetParameters_.profileType);
  if (footstep.getAverageVelocity() == 0.0)
    footstep.setAverageVelocity(footTargetParameters_.averageVelocity);
  return true;
}

bool StepCompleter::complete(BaseAuto& baseAuto) const
{
  if (baseAuto.getHeight() == 0.0)
    baseAuto.setHeight(baseAutoParameters_.height);
  if (baseAuto.getAverageVelocity() == 0.0)
    baseAuto.setAverageVelocity(baseAutoParameters_.averageVelocity);
  return true;
}

} /* namespace */

