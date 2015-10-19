/*
 * FootTargetCompleter.hpp
 *
 *  Created on: Oct 19, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/leg_motion/FootTarget.hpp"
#include "free_gait_core/TypeDefs.hpp"

// STD
#include <string>

namespace free_gait {

class FootTargetCompleter
{
 public:
  FootTargetCompleter();
  virtual ~FootTargetCompleter();

  virtual bool complete(FootTarget& footTarget) const;

 protected:
  std::string profileType_;
  double profileHeight_;
  double averageVelocity_;
  Vector surfaceNormal_;
};

} /* namespace */
