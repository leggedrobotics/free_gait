/*
 * StepCompleter.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/step/Step.hpp"
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/completers/StepCompleter.hpp"
#include "free_gait_core/completers/FootTargetCompleter.hpp"
#include "free_gait_core/completers/BaseAutoCompleter.hpp"

namespace free_gait {

class StepCompleter
{
 public:
  StepCompleter();
  virtual ~StepCompleter();
  virtual bool complete(Step& step) const;

 private:
  FootTargetCompleter footTargetCompleter_;
  BaseAutoCompleter baseAutoCompleter_;
};

} /* namespace */
