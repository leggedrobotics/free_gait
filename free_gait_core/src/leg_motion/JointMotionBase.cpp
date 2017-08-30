/*
 * JointMotionBase.cpp
 *
 *  Created on: Oct 21, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/leg_motion/JointMotionBase.hpp>

namespace free_gait {

JointMotionBase::JointMotionBase(LegMotionBase::Type type, LimbEnum limb)
    : LegMotionBase(type, limb)
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

const std::vector<JointNodeEnum> JointMotionBase::getJointNodeEnums() const
{
  throw std::runtime_error("JointMotionBase::getJointNodeEnums() not implemented.");
}

void JointMotionBase::updateStartPosition(const JointPositionsLeg& startPosition)
{
  throw std::runtime_error("JointMotionBase::updateStartPosition() not implemented.");
}

void JointMotionBase::updateStartVelocity(const JointVelocitiesLeg& startVelocity)
{
  throw std::runtime_error("JointMotionBase::updateStartVelocity() not implemented.");
}

void JointMotionBase::updateStartAcceleration(const JointAccelerationsLeg& startAcceleration)
{
  throw std::runtime_error("JointMotionBase::updateStartAcceleration() not implemented.");
}

void JointMotionBase::updateStartEfforts(const JointEffortsLeg& startEffort)
{
  throw std::runtime_error("JointMotionBase::updateStartEfforts() not implemented.");
}

const JointPositionsLeg JointMotionBase::evaluatePosition(const double time) const
{
  throw std::runtime_error("JointMotionBase::evaluatePosition() not implemented.");
}

const JointVelocitiesLeg JointMotionBase::evaluateVelocity(const double time) const
{
  throw std::runtime_error("JointMotionBase::evaluateVelocity() not implemented.");
}

const JointAccelerationsLeg JointMotionBase::evaluateAcceleration(const double time) const
{
  throw std::runtime_error("JointMotionBase::evaluateAcceleration() not implemented.");
}

const JointEffortsLeg JointMotionBase::evaluateEffort(const double time) const
{
  throw std::runtime_error("JointMotionBase::evaluateEffort() not implemented.");
}

bool JointMotionBase::isIgnoreForPoseAdaptation() const
{
  return true;
}

} /* namespace */
