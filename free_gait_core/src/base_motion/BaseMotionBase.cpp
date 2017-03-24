/*
 * BaseMotionBase.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/base_motion/BaseMotionBase.hpp"
#include "free_gait_core/base_motion/BaseAutoStepWiseBasicAlignment.hpp"
#include "free_gait_core/base_motion/BaseTarget.hpp"
#include "free_gait_core/base_motion/BaseTrajectory.hpp"

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
  throw std::runtime_error("BaseMotionBase::clone() not implemented.");
}

BaseMotionBase::Type BaseMotionBase::getType() const
{
  return type_;
}

const ControlSetup BaseMotionBase::getControlSetup() const
{
  throw std::runtime_error("BaseMotionBase::getControlSetup() not implemented.");
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

void BaseMotionBase::updateStartForceTorque(const Force& startForce, const Torque& startTorque)
{
  throw std::runtime_error("BaseMotionBase::updateStartForce() not implemented.");
}

bool BaseMotionBase::prepareComputation(const State& state, const Step& step, const StepQueue& queue, const AdapterBase& adapter)
{
  throw std::runtime_error("BaseMotionBase::prepareComputation() not implemented.");
}

bool BaseMotionBase::needsComputation() const
{
  throw std::runtime_error("BaseMotionBase::needsComputation() not implemented.");
}

bool BaseMotionBase::compute()
{
  throw std::runtime_error("BaseMotionBase::compute() not implemented.");
}

bool BaseMotionBase::isComputed() const
{
  throw std::runtime_error("BaseMotionBase::isComputed() not implemented.");
}

double BaseMotionBase::getDuration() const
{
  throw std::runtime_error("BaseMotionBase::getDuration() not implemented.");
}

const std::string& BaseMotionBase::getFrameId(const ControlLevel& controlLevel) const
{
  throw std::runtime_error("BaseMotionBase::getFrameId() not implemented.");
}

Pose BaseMotionBase::evaluatePose(const double time) const
{
  throw std::runtime_error("BaseMotionBase::evaluatePose() not implemented.");
}

Twist BaseMotionBase::evaluateTwist(const double time) const
{
  throw std::runtime_error("BaseMotionBase::evaluateTwist() not implemented.");
}

Twist BaseMotionBase::evaluateAcceleration(const double time) const
{
  throw std::runtime_error("BaseMotionBase::evaluateAcceleration() not implemented.");
}

Force BaseMotionBase::evaluateForce(const double time) const
{
  throw std::runtime_error("BaseMotionBase::evaluateForce() not implemented.");
}

Torque BaseMotionBase::evaluateTorque(const double time) const
{
  throw std::runtime_error("BaseMotionBase::evaluateTorque() not implemented.");
}

std::ostream& operator<< (std::ostream& out, const BaseMotionBase& baseMotion)
{
  out << "Type: " << baseMotion.getType() << std::endl;
  switch (baseMotion.getType()) {
    case BaseMotionBase::Type::Auto:
      out << (dynamic_cast<const BaseAuto&>(baseMotion)) << std::endl;
      break;
    case BaseMotionBase::Type::Target:
      out << (dynamic_cast<const BaseTarget&>(baseMotion)) << std::endl;
      break;
    case BaseMotionBase::Type::Trajectory:
      out << (dynamic_cast<const BaseTrajectory&>(baseMotion)) << std::endl;
      break;
    default:
      throw std::runtime_error("BaseMotionBase::operator<< not implemented for this type.");
      break;
  }
  return out;
}

std::ostream& operator<< (std::ostream& out, const BaseMotionBase::Type& type)
{
  switch (type) {
    case BaseMotionBase::Type::Auto:
      out << "Auto";
      return out;
    case BaseMotionBase::Type::Target:
      out << "Target";
      return out;
    case BaseMotionBase::Type::Trajectory:
      out << "Trajectory";
      return out;
    default:
      out << "Undefined";
      return out;
  }
}

} /* namespace */
