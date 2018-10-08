/*
 * EndEffectorTarget.hpp
 *
 *  Created on: Apr 18, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/leg_motion/EndEffectorTarget.hpp>
#include <free_gait_core/leg_motion/LegMotionBase.hpp>

// Curves
#include <curves/PolynomialSplineContainer.hpp>

namespace free_gait {

EndEffectorTarget::EndEffectorTarget(LimbEnum limb)
    : EndEffectorMotionBase(LegMotionBase::Type::EndEffectorTarget, limb),
      duration_(0.0),
      minimumDuration_(0.0),
      averageVelocity_(0.0),
      useAverageVelocity_(true),
      ignoreContact_(false),
      ignoreForPoseAdaptation_(false),
      isComputed_(false),
      controlSetup_ { {ControlLevel::Position, false}, {ControlLevel::Velocity, false},
                            {ControlLevel::Acceleration, false}, {ControlLevel::Effort, false} }
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
  if (!controlSetup_[ControlLevel::Position]) {
    std::cout << "[EndEffectorTarget::updateStartPosition] Desired control level is not in control setup.\n";
  }
  isComputed_ = false;
  startPosition_ = startPosition;
}

void EndEffectorTarget::updateStartVelocity(const LinearVelocity& startVelocity)
{
  if (!controlSetup_[ControlLevel::Velocity]) {
    std::cout << "[EndEffectorTarget::updateStartVelocity] Desired control level is not in control setup.\n";
  }
  isComputed_ = false;
  startVelocity_ = startVelocity;
}

void EndEffectorTarget::updateStartEndEffectorForce(const Force& startForce) {
  if (!controlSetup_[ControlLevel::Effort]) {
    std::cout << "[EndEffectorTarget::updateStartEndEffectorForce] Desired control level is not in control setup.\n";
  }
  isComputed_ = false;
  startForce_ = startForce;
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

void EndEffectorTarget::reset()
{
  startPosition_.setZero();
  startVelocity_.setZero();
  startForce_.setZero();
  trajectory_.clear();
  isComputed_ = false;
}

const Position EndEffectorTarget::evaluatePosition(const double time) const
{
  if (!controlSetup_.at(ControlLevel::Position)) {
    std::cout  << "[EndEffectorTarget::evaluatePosition] Desired control level is not in control setup.\n";
  }
  const double timeInRange = mapTimeWithinDuration(time);
  Position position;
  trajectory_.evaluate(position.toImplementation(), timeInRange);
  return position;
}

const LinearVelocity EndEffectorTarget::evaluateVelocity(const double time) const
{
  if (!controlSetup_.at(ControlLevel::Velocity)) {
    std::cout  << "[EndEffectorTarget::evaluateVelocity] Desired control level is not in control setup.\n";
  }
  const double timeInRange = mapTimeWithinDuration(time);
  LinearVelocity velocity;
  trajectory_.evaluateDerivative(velocity.toImplementation(), timeInRange, 1);
  return velocity;
}

const LinearAcceleration EndEffectorTarget::evaluateAcceleration(const double time) const
{
  if (!controlSetup_.at(ControlLevel::Acceleration)) {
    std::cout  << "[EndEffectorTarget::evaluateAcceleration] Desired control level is not in control setup.\n";
  }
  const double timeInRange = mapTimeWithinDuration(time);
  LinearAcceleration acceleration;
  trajectory_.evaluateDerivative(acceleration.toImplementation(), timeInRange, 2);
  return acceleration;
}

const Force EndEffectorTarget::evaluateEndEffectorForce(const double time) const
{
  if (!controlSetup_.at(ControlLevel::Effort)) {
    std::cout  << "[EndEffectorTarget::evaluateEndEffectorForce] Desired control level is not in control setup.\n";
  }

  // Linearly interpolate between start and end-force.
  if (duration_>0.0) {
    return (startForce_ + (targetForce_ - startForce_) * time/duration_);
  }
  return startForce_;
}

double EndEffectorTarget::getDuration() const
{
  return duration_;
}

void EndEffectorTarget::setTargetPosition(const std::string& frameId, const Position& targetPosition)
{
  controlSetup_[ControlLevel::Position] = true;
  frameIds_[ControlLevel::Position] = frameId;
  targetPosition_ =  targetPosition;
}

void EndEffectorTarget::setTargetVelocity(const std::string& frameId, const LinearVelocity& targetVelocity)
{
  controlSetup_[ControlLevel::Velocity] = true;
  frameIds_[ControlLevel::Velocity] = frameId;
  targetVelocity_ = targetVelocity;
}

const Position EndEffectorTarget::getTargetPosition() const
{
  if (!controlSetup_.at(ControlLevel::Position)) {
    std::cout  << "[EndEffectorTarget::getTargetPosition] Desired control level is not in control setup.\n";
  }
  return targetPosition_;
}

const LinearVelocity EndEffectorTarget::getTargetVelocity() const
{
  if (!controlSetup_.at(ControlLevel::Velocity)) {
    std::cout  << "[EndEffectorTarget::getTargetVelocity] Desired control level is not in control setup.\n";
  }
  return targetVelocity_;
}

const std::string& EndEffectorTarget::getFrameId(const ControlLevel& controlLevel) const
{
  if (!controlSetup_.at(controlLevel)) {
    std::cout  << "[EndEffectorTarget::getFrameId] Desired control level is not in control setup.\n";
  }
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

void EndEffectorTarget::setAverageVelocity(const double averageVelocity)
{
  averageVelocity_ = averageVelocity;
}

void EndEffectorTarget::computeDuration()
{
  if(controlSetup_[ControlLevel::Position] && averageVelocity_>0.0 && useAverageVelocity_) {
    double distance = (targetPosition_ - startPosition_).norm();
    double duration = distance / averageVelocity_;
    duration_ = duration < minimumDuration_ ? minimumDuration_ : duration;
  }
  else if (duration_<= 0.0) {
    std::cout  << "[EndEffectorTarget::computeDuration] Duration is smaller than zero.\n";
  }
}

bool EndEffectorTarget::computeTrajectory()
{
  if (controlSetup_[ControlLevel::Position]) {
    std::vector<Time> times;
    std::vector<ValueType> values;

    times.push_back(0.0);
    values.push_back(startPosition_.vector());

    times.push_back(duration_);
    values.push_back(targetPosition_.vector());

    // Curves implementation provides velocities and accelerations.
    controlSetup_[ControlLevel::Velocity] = true;
    frameIds_[ControlLevel::Velocity] = frameIds_[ControlLevel::Position];
    controlSetup_[ControlLevel::Acceleration] = true;
    frameIds_[ControlLevel::Acceleration] = frameIds_[ControlLevel::Position];

    trajectory_.fitCurveWithDerivatives(times, values, startVelocity_.vector(), targetVelocity_.vector());
  }
  return true;
}

const Vector& EndEffectorTarget::getImpedanceGain(const ImpedanceControl& impedanceControlId) const {
  return impedanceGains_.at(impedanceControlId);
}

const std::string& EndEffectorTarget::getImpedanceGainFrameId(const ImpedanceControl& impedanceControlId) const {
  return impedanceGainsFrameId_.at(impedanceControlId);
}


std::ostream& operator<<(std::ostream& out, const EndEffectorTarget& endEffectorTarget)
{
  if (endEffectorTarget.getControlSetup().at(ControlLevel::Position)) {
    out << "Position frame: "  << endEffectorTarget.getFrameId(ControlLevel::Position) << std::endl;
    out << "Start Position: "  << endEffectorTarget.startPosition_ << std::endl;
    out << "Target Position: " << endEffectorTarget.getTargetPosition() << std::endl;
  }
  if (endEffectorTarget.getControlSetup().at(ControlLevel::Velocity)) {
    out << "Velocity frame: "  << endEffectorTarget.getFrameId(ControlLevel::Velocity) << std::endl;
    out << "Start Velocity: "  << endEffectorTarget.startVelocity_ << std::endl;
    out << "Target Velocity: " << endEffectorTarget.getTargetVelocity() << std::endl;
  }
  if (endEffectorTarget.getControlSetup().at(ControlLevel::Effort)) {
    out << "Force frame: "  << endEffectorTarget.getFrameId(ControlLevel::Effort) << std::endl;
    out << "Start Force: "  << endEffectorTarget.startForce_ << std::endl;
    out << "Target Force: " << endEffectorTarget.targetForce_ << std::endl;
  }
  return out;
}

} /* namespace */
