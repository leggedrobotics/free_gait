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

namespace free_gait {

class StepCompleter
{
 public:
  StepCompleter();
  virtual ~StepCompleter();
  virtual bool complete(Step& step) const;
};

} /* namespace */
