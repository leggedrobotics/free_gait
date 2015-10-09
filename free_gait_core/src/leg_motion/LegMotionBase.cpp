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

double LegMotionBase::getDuration() const
{
  return 0.0;
}
//
//const std::string& SwingTrajectoryBase::getFrameId() const
//{
//  return frameId_;
//}
//
//void SwingTrajectoryBase::setFrameId(const std::string& frameId)
//{
//  frameId_ = frameId;
//}

} /* namespace */
