/*
 * JointTrajectory.cpp
 *
 *  Created on: Nov 8, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/leg_motion/JointTrajectory.hpp>

namespace free_gait {

JointTrajectory::JointTrajectory(LimbEnum limb)
    : JointMotionBase(LegMotionBase::Type::JointTrajectory, limb),
      ignoreContact_(false),
      duration_(0.0),
      isComputed_(false),
      controlSetup_ { {ControlLevel::Position, false}, {ControlLevel::Velocity, false},
                            {ControlLevel::Acceleration, false}, {ControlLevel::Effort, false} }
{
}

JointTrajectory::~JointTrajectory()
{
}

std::unique_ptr<LegMotionBase> JointTrajectory::clone() const
{
  std::unique_ptr<LegMotionBase> pointer(new JointTrajectory(*this));
  return pointer;
}

void JointTrajectory::setTrajectory(
    const std::unordered_map<ControlLevel, std::vector<Time>, EnumClassHash>& times,
    const std::unordered_map<ControlLevel, std::vector<std::vector<ValueType>>, EnumClassHash>& values,
    const std::vector<JointNodeEnum>& jointNodeEnums)
{
  times_ = times;
  values_ = values;
  jointNodeEnums_ = jointNodeEnums;
  for (const auto& value : values) controlSetup_[value.first] = true;
}

const std::vector<JointNodeEnum> JointTrajectory::getJointNodeEnums() const
{
  return jointNodeEnums_;
}

const ControlSetup JointTrajectory::getControlSetup() const
{
  return controlSetup_;
}

void JointTrajectory::updateStartPosition(const JointPositionsLeg& startPosition)
{
  isComputed_ = false;
  auto& values = values_.at(ControlLevel::Position);
  auto& times = times_.at(ControlLevel::Position);
  if (times[0] == 0.0) {
    for (size_t i = 0; i < values.size(); ++i) {
      values[i][0] = startPosition(i);
    }
  } else {
    times.insert(times.begin(), 0.0);
    for (size_t i = 0; i < values.size(); ++i) {
      values[i].insert(values[i].begin(), startPosition(i));
    }
  }
}

void JointTrajectory::updateStartVelocity(const JointVelocitiesLeg& startVelocity)
{
  throw std::runtime_error("JointTrajectory::updateStartVelocity() not implemented.");
}

void JointTrajectory::updateStartAcceleration(const JointAccelerationsLeg& startAcceleration)
{
  throw std::runtime_error("JointTrajectory::updateStartAcceleration() not implemented.");
}

void JointTrajectory::updateStartEfforts(const JointEffortsLeg& startEffort)
{
  isComputed_ = false;
  auto& values = values_.at(ControlLevel::Effort);
  auto& times = times_.at(ControlLevel::Effort);
  if (times[0] == 0.0) {
    for (size_t i = 0; i < values.size(); ++i) {
      values[i][0] = startEffort(i);
    }
  } else {
    times.insert(times.begin(), 0.0);
    for (size_t i = 0; i < values.size(); ++i) {
      values[i].insert(values[i].begin(), startEffort(i));
    }
  }
}

bool JointTrajectory::prepareComputation(const State& state, const Step& step, const AdapterBase& adapter)
{
  isComputed_ = false;
  duration_ = 0.0;
  for (const auto& times : times_) {
    if (times.second.back() > duration_) duration_ = times.second.back();
  }
  return true;
}

bool JointTrajectory::needsComputation() const
{
  return true;
}

bool JointTrajectory::compute()
{
  for (auto& trajectories : trajectories_) {
    trajectories.second.resize(values_.at(trajectories.first).size());
    for (size_t i = 0; i < values_.at(trajectories.first).size(); ++i) {
      trajectories.second[i].fitCurve(times_.at(trajectories.first), values_.at(trajectories.first)[i]);
    }
  }

  if (controlSetup_[ControlLevel::Position]) {
    // Curves implementation provides velocities and accelerations.
    controlSetup_[ControlLevel::Velocity] = true;
    controlSetup_[ControlLevel::Acceleration] = true;
  }

  isComputed_ = true;
  return true;
}

bool JointTrajectory::isComputed() const
{
  return isComputed_;
}

void JointTrajectory::reset()
{
  for (auto& trajectories : trajectories_) {
    trajectories.second.clear();
  }
  duration_ = 0.0;
  isComputed_ = false;
}

double JointTrajectory::getDuration() const
{
  return duration_;
}

const JointPositionsLeg JointTrajectory::evaluatePosition(const double time) const
{
  if (!isComputed_) throw std::runtime_error("JointTrajectory::evaluatePosition() cannot be called if trajectory is not computed.");
  const double timeInRange = mapTimeWithinDuration(time);
  const auto& trajectories = trajectories_.at(ControlLevel::Position);
  JointPositionsLeg jointPositions;
  for (size_t i = 0; i < trajectories.size(); ++i) {
    trajectories[i].evaluate(jointPositions(i), timeInRange);
  }
  return jointPositions;
}

const JointVelocitiesLeg JointTrajectory::evaluateVelocity(const double time) const
{
  if (!isComputed_) throw std::runtime_error("JointTrajectory::evaluateVelocity() cannot be called if trajectory is not computed.");
  const double timeInRange = mapTimeWithinDuration(time);
  const auto& trajectories = trajectories_.at(ControlLevel::Position);
  JointVelocitiesLeg jointVelocities;
  for (size_t i = 0; i < trajectories.size(); ++i) {
    trajectories[i].evaluateDerivative(jointVelocities(i), timeInRange, 1);
  }
  return jointVelocities;
}

const JointAccelerationsLeg JointTrajectory::evaluateAcceleration(const double time) const
{
  if (!isComputed_) throw std::runtime_error("JointTrajectory::evaluateAcceleration() cannot be called if trajectory is not computed.");
  const double timeInRange = mapTimeWithinDuration(time);
  const auto& trajectories = trajectories_.at(ControlLevel::Position);
  JointAccelerationsLeg jointAccelerations;
  for (size_t i = 0; i < trajectories.size(); ++i) {
    trajectories[i].evaluateDerivative(jointAccelerations(i), timeInRange, 2);
  }
  return jointAccelerations;
}

const JointEffortsLeg JointTrajectory::evaluateEffort(const double time) const
{
  if (!isComputed_) throw std::runtime_error("JointTrajectory::evaluateEffort() cannot be called if trajectory is not computed.");
  const auto& trajectories = trajectories_.at(ControlLevel::Effort);
  JointEffortsLeg jointEfforts;
  for (size_t i = 0; i < trajectories.size(); ++i) {
    trajectories[i].evaluate(jointEfforts(i), time);
  }
  return jointEfforts;
}

bool JointTrajectory::isIgnoreContact() const
{
  return ignoreContact_;
}

std::ostream& operator<<(std::ostream& out, const JointTrajectory& jointTrajectory)
{
  if (!jointTrajectory.isComputed()) throw std::runtime_error("JointTrajectory::operator<< cannot be called if trajectory is not computed.");
  out << "Joint nodes (" << jointTrajectory.jointNodeEnums_.size() << ")";
//  for (const auto& jointNode : jointTrajectory.jointNodeEnums_) {
//    out << jointNode << ", ";
//  }
  out << std::endl;
  out << "Ignore contact: " << (jointTrajectory.ignoreContact_ ? "True" : "False") << std::endl;
  for (const auto& times : jointTrajectory.times_) {
    out << "Times (" << times.first << "): ";
    for (const auto& time : times.second) out << time << ", ";
    out << std::endl;
  }
  return out;
}

} /* namespace */
