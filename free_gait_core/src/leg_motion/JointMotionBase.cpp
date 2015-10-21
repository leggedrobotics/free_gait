/*
 * JointMotionBase.cpp
 *
 *  Created on: Oct 21, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/leg_motion/JointMotionBase.hpp>

namespace free_gait {

JointMotionBase::JointMotionBase(LegMotionBase::Type type)
    : LegMotionBase(type)
{
}

JointMotionBase::~JointMotionBase()
{
}

std::unique_ptr<LegMotionBase> JointMotionBase::clone() const
{
  std::unique_ptr<LegMotionBase> pointer(new JointMotionBase(*this));
  return pointer;
}

LegMotionBase::TrajectoryType JointMotionBase::getTrajectoryType() const
{
  return LegMotionBase::TrajectoryType::Joints;
}

void JointMotionBase::updateStartPosition(const JointPositions& startPosition)
{
  throw std::runtime_error("JointMotionBase::updateStartPosition() not implemented.");
}

void JointMotionBase::updateStartVelocity(const JointVelocities& startVelocity)
{
  throw std::runtime_error("JointMotionBase::updateStartVelocity() not implemented.");
}

void JointMotionBase::updateStartAcceleration(const JointAccelerations& startAcceleration)
{
  throw std::runtime_error("JointMotionBase::updateStartAcceleration() not implemented.");
}

void JointMotionBase::updateStartEfforts(const JointEfforts& startEffort)
{
  throw std::runtime_error("JointMotionBase::updateStartEfforts() not implemented.");
}

const JointPositions JointMotionBase::evaluatePosition(const double time)
{
  throw std::runtime_error("JointMotionBase::evaluatePosition() not implemented.");
}

const JointVelocities JointMotionBase::evaluateVelocity(const double time)
{
  throw std::runtime_error("JointMotionBase::evaluateVelocity() not implemented.");
}

const JointAccelerations JointMotionBase::evaluateAcceleration(const double time)
{
  throw std::runtime_error("JointMotionBase::evaluateAcceleration() not implemented.");
}

const JointEfforts JointMotionBase::evaluateEffort(const double time)
{
  throw std::runtime_error("JointMotionBase::evaluateEffort() not implemented.");
}

} /* namespace */
