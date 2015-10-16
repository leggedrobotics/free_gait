/*
 * StepCompleter.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/completer/StepCompleter.hpp"
#include "free_gait_core/leg_motion/FootTarget.hpp"

// STD
#include <memory>

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
        dynamic_cast<FootTarget&>(legMotion.second);
        break;
      default:
        break;
    }

  }

  for (auto& baseMotion : step.baseMotions_) {
  }

  return step.isComplete_ = true;
}

} /* namespace */

