/*
 * EndEffectorMotionBase.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/leg_motion/EndEffectorMotionBase.hpp>

namespace free_gait {

EndEffectorMotionBase::EndEffectorMotionBase(LegMotionBase::Type type, LimbEnum limb)
    : LegMotionBase(type, limb)
{
}

EndEffectorMotionBase::~EndEffectorMotionBase()
{
}

LegMotionBase::TrajectoryType EndEffectorMotionBase::getTrajectoryType() const
{
  return LegMotionBase::TrajectoryType::EndEffector;
}

void EndEffectorMotionBase::updateStartPosition(const Position& startPosition)
{
  throw std::runtime_error("EndEffectorMotionBase::updateStartPose() not implemented.");
}

void EndEffectorMotionBase::updateStartVelocity(const LinearVelocity& startVelocity)
{
  throw std::runtime_error("EndEffectorMotionBase::updateStartVelocity() not implemented.");
}

void EndEffectorMotionBase::updateStartAcceleration(const LinearAcceleration& startAcceleration)
{
  throw std::runtime_error("EndEffectorMotionBase::updateStartAcceleration() not implemented.");
}

void EndEffectorMotionBase::updateStartEndEffectorForce(const Force& startForce)
{
  throw std::runtime_error("EndEffectorMotionBase::updateStartEndEffectorForce() not implemented.");
}

const Position EndEffectorMotionBase::evaluatePosition(const double time) const
{
  throw std::runtime_error("EndEffectorMotionBase::evaluatePosition() not implemented.");
}

const LinearVelocity EndEffectorMotionBase::evaluateVelocity(const double time) const
{
  throw std::runtime_error("EndEffectorMotionBase::evaluateVelocity() not implemented.");
}

const LinearAcceleration EndEffectorMotionBase::evaluateAcceleration(const double time) const
{
  throw std::runtime_error("EndEffectorMotionBase::evaluateAcceleration() not implemented.");
}

const Force EndEffectorMotionBase::evaluateEndEffectorForce(const double time) const
{
  throw std::runtime_error("EndEffectorMotionBase::evaluateForce() not implemented.");
}

const Position EndEffectorMotionBase::getTargetPosition() const
{
  throw std::runtime_error("EndEffectorMotionBase::getTargetPosition() not implemented.");
}

const LinearVelocity EndEffectorMotionBase::getTargetVelocity() const
{
  throw std::runtime_error("EndEffectorMotionBase::getTargetVelocity() not implemented.");
}

const std::string& EndEffectorMotionBase::getFrameId(const ControlLevel& controlLevel) const
{
  throw std::runtime_error("EndEffectorMotionBase::getFrameId() not implemented.");
}

const Vector& EndEffectorMotionBase::getImpedancePositionGain() const {
  throw std::runtime_error("EndEffectorMotionBase::getImpedancePositionGain() not implemented.");
}

const Vector& EndEffectorMotionBase::getImpedanceVelocityGain() const {
  throw std::runtime_error("EndEffectorMotionBase::getImpedanceVelocityGain() not implemented.");
}

const Vector& EndEffectorMotionBase::getImpedanceForceGain() const {
  throw std::runtime_error("EndEffectorMotionBase::getImpedanceForceGain() not implemented.");
}

} /* namespace */
