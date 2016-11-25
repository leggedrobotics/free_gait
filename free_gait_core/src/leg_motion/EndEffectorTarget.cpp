/*
 * EndEffectorTarget.hpp
 *
 *  Created on: Apr 18, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/leg_motion/EndEffectorTarget.hpp>
#include <free_gait_core/leg_motion/LegMotionBase.hpp>

// Robot utils
#include <robot_utils/function_approximators/polynomialSplines/PolynomialSplineContainer.hpp>

namespace free_gait {

EndEffectorTarget::EndEffectorTarget(LimbEnum limb)
    : EndEffectorMotionBase(LegMotionBase::Type::EndEffectorTarget, limb),
      duration_(0.0),
      minimumDuration_(0.0),
      averageVelocity_(0.0),
      ignoreContact_(false),
      ignoreForPoseAdaptation_(false),
      isComputed_(false)
{
}

EndEffectorTarget::~EndEffectorTarget()
{
}

std::unique_ptr<LegMotionBase> EndEffectorTarget::clone() const
{
  std::unique_ptr<LegMotionBase> pointer(new EndEffectorTarget(*this));
  return pointer;
}

const ControlSetup EndEffectorTarget::getControlSetup() const
{
  return controlSetup_;
}

void EndEffectorTarget::updateStartPosition(const Position& startPosition)
{
  isComputed_ = false;
  start_[ControlLevel::Position] = startPosition.vector();
}

bool EndEffectorTarget::prepareComputation(const State& state, const Step& step, const AdapterBase& adapter)
{
  computeDuration();
  return isComputed_ = computeTrajectory();
}

bool EndEffectorTarget::needsComputation() const
{
  return false;
}

bool EndEffectorTarget::isComputed() const
{
  return isComputed_;
}

const Position EndEffectorTarget::evaluatePosition(const double time) const
{
  Position position;
  trajectory_.evaluate(position.toImplementation(), time);
  return position;
}

double EndEffectorTarget::getDuration() const
{
  return duration_;
}

const Position EndEffectorTarget::getTargetPosition() const
{
  return Position(target_.at(ControlLevel::Position));
}

const std::string& EndEffectorTarget::getFrameId(const ControlLevel& controlLevel) const
{
  return frameIds_.at(controlLevel);
}

bool EndEffectorTarget::isIgnoreContact() const
{
  return ignoreContact_;
}

bool EndEffectorTarget::isIgnoreForPoseAdaptation() const
{
  return ignoreForPoseAdaptation_;
}

void EndEffectorTarget::computeDuration()
{
    double distance = (target_.at(ControlLevel::Position) - start_.at(ControlLevel::Position)).norm();
    double duration = distance / averageVelocity_;
    duration_ = duration < minimumDuration_ ? minimumDuration_ : duration;
}

bool EndEffectorTarget::computeTrajectory()
{
  std::vector<Time> times;
  std::vector<ValueType> values;

  times.push_back(0.0);
  values.push_back(start_.at(ControlLevel::Position));

  times.push_back(duration_);
  values.push_back(target_.at(ControlLevel::Position));

  trajectory_.fitCurve(times, values);
  return true;
}

std::ostream& operator<<(std::ostream& out, const EndEffectorTarget& endEffectorTarget)
{
  out << "Duration: " << endEffectorTarget.getDuration() << std::endl;
  out << "Ignore contact: " << (endEffectorTarget.isIgnoreContact() ? "True" : "False") << std::endl;
  out << "Ignore for pose adaptation: " << (endEffectorTarget.isIgnoreForPoseAdaptation() ? "True" : "False") << std::endl;
  return out;
}

} /* namespace */
