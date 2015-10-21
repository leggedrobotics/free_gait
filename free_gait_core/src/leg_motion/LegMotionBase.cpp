/*
 * LegMotionBase.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/leg_motion/LegMotionBase.hpp>

namespace free_gait {

LegMotionBase::LegMotionBase(LegMotionBase::Type type)
    : type_(type)
{
}

LegMotionBase::~LegMotionBase()
{
}

std::unique_ptr<LegMotionBase> LegMotionBase::clone() const
{
  std::unique_ptr<LegMotionBase> pointer(new LegMotionBase(*this));
  return pointer;
}

LegMotionBase::Type LegMotionBase::getType() const
{
  return type_;
}

LegMotionBase::TrajectoryType LegMotionBase::getTrajectoryType() const
{
  throw std::runtime_error("LegMotionBase::getTrajectoryType() not implemented.");
}

const ControlSetup LegMotionBase::getControlSetup() const
{
  throw std::runtime_error("LegMotionBase::getControlSetup() not implemented.");
}

double LegMotionBase::getDuration() const
{
  throw std::runtime_error("LegMotionBase::getDuration() not implemented.");
}

const Vector& LegMotionBase::getSurfaceNormal() const
{
  throw std::runtime_error("LegMotionBase::getSurfaceNormal() not implemented.");
}

bool LegMotionBase::isIgnoreContact() const
{
  throw std::runtime_error("LegMotionBase::isIgnoreContact() not implemented.");
}

bool LegMotionBase::isIgnoreForPoseAdaptation() const
{
  throw std::runtime_error("LegMotionBase::isIgnoreForPoseAdaptation() not implemented.");
}

} /* namespace */
