/*
 * StepCompleter.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/completer/FootTargetCompleter.hpp"
#include "free_gait_core/SwingProfile.hpp"
#include "free_gait_core/BaseShiftProfile.hpp"

// STD
#include <memory>

namespace free_gait {

FootTargetCompleter::FootTargetCompleter()
    : swingProfileDuration_(2.0),
      swingProfileHeight_(0.06),
      swingProfileType_("triangle"),
      standardBaseShiftProfileDuration_(2.0),
      baseShiftProfileHeight_(0.46),
      baseShiftProfileType_("linear"),
      surfaceNormal_(Vector::UnitZ())
{
}

FootTargetCompleter::~FootTargetCompleter()
{
}

bool FootTargetCompleter::complete(Step& step) const
{
  for (auto& swingData : step.legMotion_) {
    // Surface normal.
    const auto& normal = swingData.second.getSurfaceNormal();
    if (normal == Vector::Zero()) swingData.second.setSurfaceNormal(surfaceNormal_);

    // Profile.
    if (swingData.second.getTrajectory()->getType() == SwingTrajectoryType::Profile) {
      auto profile = std::static_pointer_cast<SwingProfile>(swingData.second.getTrajectory());
      if (profile->getDuration() == 0.0) profile->setDuration(swingProfileDuration_);
      if (profile->getHeight() == 0.0) profile->setHeight(swingProfileHeight_);
      if (profile->getProfileType().empty()) profile->setProfileType(swingProfileType_);
    }
  }

  for (auto& baseShiftData : step.baseShiftData_) {
    // Profile.
    if (baseShiftData.second.getTrajectory()->getType() == BaseShiftTrajectoryType::Profile) {
      auto profile = std::static_pointer_cast<BaseShiftProfile>(baseShiftData.second.getTrajectory());
      if (profile->getDuration() == 0.0) profile->setDuration(standardBaseShiftProfileDuration_);
      if (profile->getHeight() == 0.0) profile->setHeight(baseShiftProfileHeight_);
      if (profile->getProfileType().empty()) profile->setProfileType(baseShiftProfileType_);
    }
  }

  return step.isComplete_ = true;
}

} /* namespace */
