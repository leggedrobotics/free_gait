/*
 * StepCompleter.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/completers/StepCompleter.hpp"
#include "free_gait_core/completers/BaseAutoCompleter.hpp"
#include "free_gait_core/completers/FootTargetCompleter.hpp"
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
  for (auto& legMotion : step.legMotions_) {
    switch (legMotion.second.getType()) {
      case LegMotionBase::Type::FootTarget:
        if (!footTargetCompleter_.complete(dynamic_cast<FootTarget&>(legMotion.second))) return false;
        break;
      default:
        break;
    }

  }

  for (auto& baseMotion : step.baseMotions_) {
    switch (baseMotion.second.getType()) {
      case BaseMotionBase::Type::Auto:
        if (!baseAutoCompleter_.complete(dynamic_cast<BaseAuto&>(baseMotion.second))) return false;
        break;
      default:
        break;
    }
  }

  return step.isComplete_ = true;
}

} /* namespace */

