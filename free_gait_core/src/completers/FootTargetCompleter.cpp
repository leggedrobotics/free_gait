/*
 * FootTargetCompleter.hpp
 *
 *  Created on: Oct 19, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/completers/FootTargetCompleter.hpp>

// STD
#include <memory>

namespace free_gait {

FootTargetCompleter::FootTargetCompleter()
    : profileType_("triangle"),
      profileHeight_(0.06),
      averageVelocity_(2.0),
      surfaceNormal_(Vector::UnitZ())
{
}

FootTargetCompleter::~FootTargetCompleter()
{
}

bool FootTargetCompleter::complete(FootTarget& footTarget) const
{
  if (footTarget.getSurfaceNormal() == Vector::Zero())
    footTarget.setSurfaceNormal(surfaceNormal_);
  if (footTarget.getProfileHeight() == 0.0)
    footTarget.setProfileHeight(profileHeight_);
  if (footTarget.getProfileType().empty())
    footTarget.setProfileType(profileType_);
  if (footTarget.getAverageVelocity() == 0.0)
    footTarget.setAverageVelocity(averageVelocity_);
  return true;
}

} /* namespace */
