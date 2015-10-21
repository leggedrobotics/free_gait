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

std::unique_ptr<BaseMotionBase> BaseMotionBase::clone() const
{
  std::unique_ptr<BaseMotionBase> pointer(new BaseMotionBase(*this));
  return pointer;
}

BaseMotionBase::Type BaseMotionBase::getType() const
{
  return type_;
}

const ControlSetup BaseMotionBase::getControlSetup() const
{
  throw std::runtime_error("BaseMotionBase::getControlSetup() not implemented.");
}

double BaseMotionBase::getDuration() const
{
  throw std::runtime_error("BaseMotionBase::getDuration() not implemented.");
}

const std::string& BaseMotionBase::getFrameId() const
{
  throw std::runtime_error("BaseMotionBase::getFrameId() not implemented.");
}

void BaseMotionBase::updateStartPose(const Pose& startPose)
{
  throw std::runtime_error("BaseMotionBase::updateStartPose() not implemented.");
}

void BaseMotionBase::updateStartTwist(const Twist& startTwist)
{
  throw std::runtime_error("BaseMotionBase::updateStartTwist() not implemented.");
}

void BaseMotionBase::updateStartAcceleration(const Twist& startAcceleration)
{
  throw std::runtime_error("BaseMotionBase::updateStartAcceleration() not implemented.");
}

void BaseMotionBase::updateStartForce(const Force& startForce, const Torque& startTorque)
{
  throw std::runtime_error("BaseMotionBase::updateStartForce() not implemented.");
}

Pose BaseMotionBase::evaluatePose(const double time)
{
  throw std::runtime_error("BaseMotionBase::evaluatePose() not implemented.");
}

Twist BaseMotionBase::evaluateTwist(const double time)
{
  throw std::runtime_error("BaseMotionBase::evaluateTwist() not implemented.");
}

Twist BaseMotionBase::evaluateAcceleration(const double time)
{
  throw std::runtime_error("BaseMotionBase::evaluateAcceleration() not implemented.");
}

Force BaseMotionBase::evaluateForce(const double time)
{
  throw std::runtime_error("BaseMotionBase::evaluateForce() not implemented.");
}

Torque BaseMotionBase::evaluateTorque(const double time)
{
  throw std::runtime_error("BaseMotionBase::evaluateTorque() not implemented.");
}

} /* namespace */
