/*
 * EndEffectorTrajectory.hpp
 *
 *  Created on: Mar 16, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/leg_motion/EndEffectorTrajectory.hpp>
#include <free_gait_core/leg_motion/LegMotionBase.hpp>

namespace free_gait {

EndEffectorTrajectory::EndEffectorTrajectory(LimbEnum limb)
    : EndEffectorMotionBase(LegMotionBase::Type::EndEffectorTrajectory, limb),
      duration_(0.0),
      ignoreContact_(false),
      ignoreForPoseAdaptation_(false),
      isComputed_(false),
      controlSetup_ { {ControlLevel::Position, false}, {ControlLevel::Velocity, false},
                      {ControlLevel::Acceleration, false}, {ControlLevel::Effort, false} }
{
}

EndEffectorTrajectory::~EndEffectorTrajectory()
{
}

std::unique_ptr<LegMotionBase> EndEffectorTrajectory::clone() const
{
  std::unique_ptr<LegMotionBase> pointer(new EndEffectorTrajectory(*this));
  return pointer;
}

void EndEffectorTrajectory::setTrajectory(
    const std::unordered_map<ControlLevel, std::string, EnumClassHash>& frameIds,
    const std::vector<Time>& times,
    const std::unordered_map<ControlLevel, std::vector<ValueType>, EnumClassHash>& values)
{
  frameIds_ = frameIds;
  times_ = times;
  values_ = values;
  for (const auto& value : values) controlSetup_[value.first] = true;
}

void EndEffectorTrajectory::setFrameId(const ControlLevel& controlLevel, const std::string& frameId)
{
  frameIds_[controlLevel] = frameId;
}

bool EndEffectorTrajectory::addPositionTrajectoryPoint(const Time& time, const Position& position)
{
  controlSetup_[ControlLevel::Position] = true;
  times_.push_back(time);
  values_[ControlLevel::Position].push_back(position.vector());
  return true;
}

const ControlSetup EndEffectorTrajectory::getControlSetup() const
{
  return controlSetup_;
}

void EndEffectorTrajectory::updateStartPosition(const Position& startPosition)
{
  isComputed_ = false;
  auto& values = values_.at(ControlLevel::Position);
  if (times_[0] == 0.0) {
    values[0] = startPosition.vector();
  } else {
    times_.insert(times_.begin(), 0.0);
    values.insert(values.begin(), startPosition.vector());
  }
}

void EndEffectorTrajectory::updateStartVelocity(const LinearVelocity& startVelocity)
{
  isComputed_ = false;
  auto& values = values_.at(ControlLevel::Velocity);
  if (times_[0] == 0.0) {
    values[0] = startVelocity.vector();
  } else {
    times_.insert(times_.begin(), 0.0);
    values.insert(values.begin(), startVelocity.vector());
  }
}

const Position EndEffectorTrajectory::getStartPosition() const
{
  return Position(values_.at(ControlLevel::Position).front());
}

const LinearVelocity EndEffectorTrajectory::getStartVelocity() const
{
  return LinearVelocity(values_.at(ControlLevel::Velocity).front());
}

bool EndEffectorTrajectory::prepareComputation(const State& state, const Step& step, const AdapterBase& adapter)
{
  duration_ = times_.back();
  if (controlSetup_[ControlLevel::Position] && !controlSetup_[ControlLevel::Velocity]) {
    trajectory_.fitCurve(times_, values_.at(ControlLevel::Position));
  } else if (controlSetup_[ControlLevel::Position] && controlSetup_[ControlLevel::Velocity]) {
    trajectory_.fitCurveWithDerivatives(times_, values_.at(ControlLevel::Position),
                                        values_.at(ControlLevel::Velocity).front(),
                                        values_.at(ControlLevel::Velocity).back());
  } // TODO Extend the options here.

  // Curves implementation provides velocities and accelerations.
  if (controlSetup_[ControlLevel::Position]) {
    controlSetup_[ControlLevel::Velocity] = true;
    frameIds_[ControlLevel::Velocity] = frameIds_[ControlLevel::Position];
    controlSetup_[ControlLevel::Acceleration] = true;
    frameIds_[ControlLevel::Acceleration] = frameIds_[ControlLevel::Position];
  }

  isComputed_ = true;
  return true;
}

bool EndEffectorTrajectory::needsComputation() const
{
  return false;
}

bool EndEffectorTrajectory::isComputed() const
{
  return isComputed_;
}

void EndEffectorTrajectory::reset()
{
  trajectory_.clear();
  duration_ = 0.0;
  isComputed_ = false;
}

const Position EndEffectorTrajectory::evaluatePosition(const double time) const
{
  const double timeInRange = mapTimeWithinDuration(time);
  Position position;
  trajectory_.evaluate(position.toImplementation(), time);
  return position;
}

const LinearVelocity EndEffectorTrajectory::evaluateVelocity(const double time) const
{
  const double timeInRange = mapTimeWithinDuration(time);
  LinearVelocity velocity;
  trajectory_.evaluateDerivative(velocity.toImplementation(), timeInRange, 1);
  return velocity;
}

const LinearAcceleration EndEffectorTrajectory::evaluateAcceleration(const double time) const
{
  const double timeInRange = mapTimeWithinDuration(time);
  LinearAcceleration acceleration;
  trajectory_.evaluateDerivative(acceleration.toImplementation(), timeInRange, 2);
  return acceleration;
}

double EndEffectorTrajectory::getDuration() const
{
  return duration_;
}

const Position EndEffectorTrajectory::getTargetPosition() const
{
  return Position(values_.at(ControlLevel::Position).back());
}

const std::string& EndEffectorTrajectory::getFrameId(const ControlLevel& controlLevel) const
{
  return frameIds_.at(controlLevel);
}

void EndEffectorTrajectory::setIgnoreContact(bool ignoreContact)
{
  ignoreContact_ = ignoreContact;
}

bool EndEffectorTrajectory::isIgnoreContact() const
{
  return ignoreContact_;
}

bool EndEffectorTrajectory::isIgnoreForPoseAdaptation() const
{
  return ignoreForPoseAdaptation_;
}

std::ostream& operator<<(std::ostream& out, const EndEffectorTrajectory& endEffectorTrajectory)
{
  out << "Ignore contact: " << (endEffectorTrajectory.isIgnoreContact() ? "True" : "False") << std::endl;
  out << "Ignore for pose adaptation: " << (endEffectorTrajectory.isIgnoreForPoseAdaptation() ? "True" : "False") << std::endl;
  out << "Times: ";
  for (const auto& time : endEffectorTrajectory.times_) out << time << ", ";
  out << std::endl;
  out << "Positions: ";
  for (const auto& position : endEffectorTrajectory.values_.at(ControlLevel::Position)) out << "[" << position.transpose() << "], ";
  out << std::endl;
  if (endEffectorTrajectory.values_.find(ControlLevel::Velocity) != endEffectorTrajectory.values_.end()) {
    out << "Velocities: ";
    for (const auto& velocity : endEffectorTrajectory.values_.at(ControlLevel::Velocity)) out << "[" << velocity.transpose() << "], ";
    out << std::endl;
  }
  return out;
}

} /* namespace */
