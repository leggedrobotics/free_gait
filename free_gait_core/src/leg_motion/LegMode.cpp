/*
 * LegMode.cpp
 *
 *  Created on: Nov 16, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/leg_motion/LegMode.hpp>
#include <free_gait_core/leg_motion/LegMotionBase.hpp>

// Roco
#include <roco/log/log_messages.hpp>

namespace free_gait {

LegMode::LegMode(LimbEnum limb)
    : EndEffectorMotionBase(LegMotionBase::Type::LegMode, limb),
      duration_(0.0),
      ignoreContact_(false),
      ignoreForPoseAdaptation_(false),
      computed_(false),
      controlSetup_ { {ControlLevel::Position, false}, {ControlLevel::Velocity, false},
                      {ControlLevel::Acceleration, false}, {ControlLevel::Effort, false} }

// TODO Should all position stuff be removed? Test on hardware first.
{
}

LegMode::~LegMode()
{
}

std::unique_ptr<LegMotionBase> LegMode::clone() const
{
  std::unique_ptr<LegMotionBase> pointer(new LegMode(*this));
  return pointer;
}

const ControlSetup LegMode::getControlSetup() const
{
  return controlSetup_;
}

void LegMode::updateStartPosition(const Position& startPosition)
{
  std::cout << "LegMode::updateStartPosition " <<  startPosition << std::endl;
  computed_ = false;
  position_ = startPosition;
}

bool LegMode::compute(const State& state, const Step& step, const AdapterBase& adapter)
{
  computed_ = true;
  return true;
}

const Position LegMode::evaluatePosition(const double time) const
{
  return position_;
}

double LegMode::getDuration() const
{
  return duration_;
}

const Position LegMode::getTargetPosition() const
{
  return position_;
}

const std::string& LegMode::getFrameId() const
{
  return frameId_;
}

bool LegMode::isIgnoreContact() const
{
  return ignoreContact_;
}

bool LegMode::isIgnoreForPoseAdaptation() const
{
  return ignoreForPoseAdaptation_;
}

std::ostream& operator<<(std::ostream& out, const LegMode& legMode)
{
  out << "Support leg: " << (!legMode.ignoreContact_ ? "True" : "False") << std::endl;
  out << "Duration: " << legMode.duration_ << std::endl;
  out << "Position: " << legMode.position_ << std::endl;
  return out;
}

} /* namespace */

