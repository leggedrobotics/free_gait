/*
 * LegMotionBase.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/leg_motion/LegMotionBase.hpp>
#include <free_gait_core/leg_motion/Footstep.hpp>
#include <free_gait_core/leg_motion/JointTrajectory.hpp>

namespace free_gait {

LegMotionBase::LegMotionBase(LegMotionBase::Type type, LimbEnum limb)
    : type_(type),
      limb_(limb)
{
}

LegMotionBase::~LegMotionBase()
{
}

LegMotionBase::LegMotionBase(const LegMotionBase& other) :
    type_(other.type_),
    limb_(other.limb_)
{
  if (other.surfaceNormal_) surfaceNormal_.reset(new Vector(*(other.surfaceNormal_)));
}

std::unique_ptr<LegMotionBase> LegMotionBase::clone() const
{
  throw std::runtime_error("LegMotionBase::clone() not implemented.");
}

LegMotionBase::Type LegMotionBase::getType() const
{
  return type_;
}

LimbEnum LegMotionBase::getLimb() const
{
  return limb_;
}

LegMotionBase::TrajectoryType LegMotionBase::getTrajectoryType() const
{
  throw std::runtime_error("LegMotionBase::getTrajectoryType() not implemented.");
}

const ControlSetup LegMotionBase::getControlSetup() const
{
  throw std::runtime_error("LegMotionBase::getControlSetup() not implemented.");
}

bool LegMotionBase::compute(const State& state, const Step& step, const AdapterBase& adapter)
{
  throw std::runtime_error("LegMotionBase::compute() not implemented.");
}

double LegMotionBase::getDuration() const
{
  throw std::runtime_error("LegMotionBase::getDuration() not implemented.");
}

bool LegMotionBase::hasSurfaceNormal() const
{
  return (bool)(surfaceNormal_);
}

const Vector& LegMotionBase::getSurfaceNormal() const
{
  if (!hasSurfaceNormal()) throw std::runtime_error("LegMotionBase::getSurfaceNormal(): No surface normal available.");
  else return (*surfaceNormal_);
}

bool LegMotionBase::isIgnoreContact() const
{
  throw std::runtime_error("LegMotionBase::isIgnoreContact() not implemented.");
}

bool LegMotionBase::isIgnoreForPoseAdaptation() const
{
  throw std::runtime_error("LegMotionBase::isIgnoreForPoseAdaptation() not implemented.");
}

std::ostream& operator<< (std::ostream& out, const LegMotionBase& legMotion)
{
  out << "Type: " << legMotion.getType() << std::endl;
  out << "Control setup: ";
  for (const auto& controlSetup : legMotion.getControlSetup()) {
    if (controlSetup.second) out << controlSetup.first;
  }
  out << std::endl;
  switch (legMotion.getType()) {
    case LegMotionBase::Type::Footstep:
      out << (dynamic_cast<const Footstep&>(legMotion)) << std::endl;
      break;
    case LegMotionBase::Type::JointTrajectory:
      out << (dynamic_cast<const JointTrajectory&>(legMotion)) << std::endl;
      break;
    default:
      throw std::runtime_error("LegMotionBase::operator<< not implemented for this type.");
      break;
  }
  return out;
}

std::ostream& operator<< (std::ostream& out, const LegMotionBase::Type& type)
{
  switch (type) {
    case LegMotionBase::Type::Footstep:
      out << "Footstep";
      return out;
    case LegMotionBase::Type::EndEffectorTarget:
      out << "EndEffectorTarget";
      return out;
    case LegMotionBase::Type::EndEffectorTrajectory:
      out << "EndEffectorTrajectory";
      return out;
    case LegMotionBase::Type::JointTarget:
      out << "JointTarget";
      return out;
    case LegMotionBase::Type::JointTrajectory:
      out << "JointTrajectory";
      return out;
    default:
      out << "Undefined";
      return out;
  }
}

} /* namespace */
