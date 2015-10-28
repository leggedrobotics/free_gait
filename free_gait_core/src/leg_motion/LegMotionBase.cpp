/*
 * LegMotionBase.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/leg_motion/LegMotionBase.hpp>
#include <free_gait_core/leg_motion/Footstep.hpp>

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
  throw std::runtime_error("LegMotionBase::clone() not implemented.");
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

std::ostream& operator<< (std::ostream& out, const LegMotionBase& legMotion)
{
  out << "Type: " << legMotion.getType() << std::endl;
  switch (legMotion.getType()) {
    case LegMotionBase::Type::Footstep:
      out << (dynamic_cast<const Footstep&>(legMotion)) << std::endl;
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
