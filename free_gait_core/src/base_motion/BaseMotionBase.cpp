/*
 * BaseMotionBase.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/base_motion/BaseMotionBase.hpp>

namespace free_gait {

BaseMotionBase::BaseMotionBase(BaseMotionBase::Type type)
    : type_(type)
{
}

BaseMotionBase::~BaseMotionBase()
{
}

BaseMotionBase::Type BaseMotionBase::getType() const
{
  return type_;
}
//
//const std::string& BaseMotionBase::getFrameId() const
//{
//  return frameId_;
//}
//
//void BaseMotionBase::setFrameId(const std::string& frameId)
//{
//  frameId_ = frameId;
//}

} /* namespace */
