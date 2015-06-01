/*
 * StepCompleter.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// Loco
#include "free_gait_core/StepCompleter.hpp"
#include "free_gait_core/SwingProfile.hpp"
#include "free_gait_core/BaseShiftProfile.hpp"

// STD
#include <memory>

namespace free_gait {

StepCompleter::StepCompleter() // TODO Load through XML.
    : swingProfileDuration_(1.5),
      swingProfileHeight_(0.1),
      swingProfileType_("triangle"),
      baseShiftProfileDuration_(1.5),
      baseShiftProfileHeight_(0.38),
      baseShiftProfileType_("linear")
{
  surfaceNormal_ = Vector::UnitZ();
}

StepCompleter::~StepCompleter()
{
}

bool StepCompleter::complete(Step& step)
{
  for (auto& swingData : step.swingData_) {
    // Surface normal.
    const auto& normal = swingData.second.getSurfaceNormal();
    if (normal == Vector::Zero()) swingData.second.setSurfaceNormal(surfaceNormal_);

    // Profile.
    if (swingData.second.isUsingProfile()) {
      auto profile = std::static_pointer_cast<SwingProfile>(swingData.second.getTrajectory());
      if (profile->getDuration() == 0.0) profile->setDuration(swingProfileDuration_);
      if (profile->getHeight() == 0.0) profile->setHeight(swingProfileHeight_);
      if (profile->getType().empty()) profile->setType(swingProfileType_);
    }
  }

  for (auto& baseShiftData : step.baseShiftData_) {
    // Profile.
    if (baseShiftData.second.isUsingProfile()) {
      auto profile = std::static_pointer_cast<BaseShiftProfile>(baseShiftData.second.getTrajectory());
      if (profile->getType() != "ignore") { // TODO?
        if (profile->getDuration() == 0.0) profile->setDuration(baseShiftProfileDuration_);
        if (profile->getHeight() == 0.0) profile->setHeight(baseShiftProfileHeight_);
        if (profile->getType().empty()) profile->setType(baseShiftProfileType_);
      }
    }
  }

  return step.isComplete_ = true;
}

} /* namespace loco */
