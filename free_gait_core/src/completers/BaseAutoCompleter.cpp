/*
 * BaseAutoCompleter.hpp
 *
 *  Created on: Oct 19, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/completers/BaseAutoCompleter.hpp>

// STD
#include <memory>

namespace free_gait {

BaseAutoCompleter::BaseAutoCompleter()
    : height_(0.46),
      averageVelocity_(2.0)
{
}

BaseAutoCompleter::~BaseAutoCompleter()
{
}

bool BaseAutoCompleter::complete(BaseAuto& baseAuto) const
{
  if (baseAuto.getHeight() == 0.0)
    baseAuto.setHeight(height_);
  if (baseAuto.getAverageVelocity() == 0.0)
    baseAuto.setAverageVelocity(averageVelocity_);
  return true;
}

} /* namespace */
