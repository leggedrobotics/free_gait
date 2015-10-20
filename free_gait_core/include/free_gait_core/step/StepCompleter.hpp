/*
 * StepCompleter.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include <free_gait_core/leg_motion/Footstep.hpp>
#include "free_gait_core/step/Step.hpp"
#include "free_gait_core/base_motion/BaseAuto.hpp"
#include "free_gait_core/TypeDefs.hpp"

namespace free_gait {

class StepCompleter
{
 public:
  StepCompleter();
  virtual ~StepCompleter();
  virtual bool complete(Step& step) const;
  virtual bool complete(Footstep& footTarget) const;
  virtual bool complete(BaseAuto& baseAuto) const;

 protected:

  struct FootstepParameters
  {
    std::string profileType = "triangle";
    Vector surfaceNormal = Vector::UnitZ();
    double profileHeight = 0.06;
    double averageVelocity = 2.0;
  } footTargetParameters_;

  struct BaseAutoParameters
  {
    double height = 0.46;
    double averageVelocity = 2.0;
  } baseAutoParameters_;

};

} /* namespace */
