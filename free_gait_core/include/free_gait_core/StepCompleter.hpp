/*
 * StepCompleter.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/Step.hpp"
#include "free_gait_core/TypeDefs.hpp"

// STD
#include <string>

namespace free_gait {

class StepCompleter
{
 public:
  StepCompleter();
  virtual ~StepCompleter();

  virtual bool complete(Step& step) const;

  Vector surfaceNormal_;
  double swingProfileDuration_;
  double swingProfileHeight_;
  std::string swingProfileType_;
  double baseShiftProfileDuration_;
  double baseShiftProfileHeight_;
  std::string baseShiftProfileType_;
};

} /* namespace loco */
