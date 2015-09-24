/*
 * BaseShiftTrajectoryBase.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/BaseShiftTrajectoryBase.hpp>

namespace free_gait {

BaseShiftTrajectoryBase::BaseShiftTrajectoryBase(BaseShiftTrajectoryType type)
    : type_(type)
{
}

BaseShiftTrajectoryBase::~BaseShiftTrajectoryBase()
{
}

BaseShiftTrajectoryType BaseShiftTrajectoryBase::getType() const
{
  return type_;
}

const std::string& BaseShiftTrajectoryBase::getFrameId() const
{
  return frameId_;
}

void BaseShiftTrajectoryBase::setFrameId(const std::string& frameId)
{
  frameId_ = frameId;
}

} /* namespace */
