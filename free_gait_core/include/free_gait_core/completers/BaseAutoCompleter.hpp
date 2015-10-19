/*
 * BaseAutoCompleter.hpp
 *
 *  Created on: Oct 19, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/base_motion/BaseAuto.hpp"
#include "free_gait_core/TypeDefs.hpp"

// STD
#include <string>

namespace free_gait {

class BaseAutoCompleter
{
 public:
  BaseAutoCompleter();
  virtual ~BaseAutoCompleter();

  virtual bool complete(BaseAuto& baseAuto) const;

 protected:
  double height_;
  double averageVelocity_;
};

} /* namespace */
