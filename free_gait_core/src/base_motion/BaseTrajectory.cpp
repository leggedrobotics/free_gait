/*
 * BaseTrajectory.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/base_motion/BaseTrajectory.hpp>

namespace free_gait {

BaseTrajectory::BaseTrajectory()
    : BaseMotionBase(BaseMotionBase::Type::Trajectory),
      duration_(0.0),
      isComputed_(false)
{
}

BaseTrajectory::~BaseTrajectory()
{
}

std::unique_ptr<BaseMotionBase> BaseTrajectory::clone() const
{
  std::unique_ptr<BaseMotionBase> pointer(new BaseTrajectory(*this));
  return pointer;
}

const ControlSetup BaseTrajectory::getControlSetup() const
{
  return controlSetup_;
}

void BaseTrajectory::updateStartPose(const Pose& startPose)
{
  isComputed_ = false;
  auto& values = values_.at(ControlLevel::Position);
  auto& times = times_.at(ControlLevel::Position);
  if (times[0] == 0.0) {
    values[0] = startPose;
  } else {
    times.insert(times.begin(), 0.0);
    values.insert(values.begin(), startPose);
  }
}

bool BaseTrajectory::prepareComputation(const State& state, const Step& step,
                                        const StepQueue& queue, const AdapterBase& adapter)
{
  trajectory_.fitCurve(times_[ControlLevel::Position], values_[ControlLevel::Position]);

  duration_ = 0.0;
  for (const auto& times : times_) {
    if (times.second.back() > duration_) duration_ = times.second.back();
  }

  isComputed_ = true;
  return true;
}

bool BaseTrajectory::needsComputation() const
{
  return false;
}

bool BaseTrajectory::isComputed() const
{
  return isComputed_;
}

double BaseTrajectory::getDuration() const
{
  return duration_;
}

const std::string& BaseTrajectory::getFrameId(const ControlLevel& controlLevel) const
{
  return frameIds_.at(controlLevel);
}

Pose BaseTrajectory::evaluatePose(const double time) const
{
  Pose pose;
  trajectory_.evaluate(pose, time);
  return pose;
}

std::ostream& operator<<(std::ostream& out, const BaseTrajectory& baseTrajectory)
{
  out << "Frame [Position]: " << baseTrajectory.frameIds_.at(ControlLevel::Position) << std::endl;
  out << "Duration: " << baseTrajectory.duration_ << std::endl;
  out << "Times: ";
  for (const auto& time : baseTrajectory.times_.at(ControlLevel::Position)) out << time << ", ";
  out << std::endl;
  out << "Positions: ";
  for (const auto& pose : baseTrajectory.values_.at(ControlLevel::Position)) out << "[" << pose.getPosition() << "], ";
  out << std::endl;
  out << "Orientations: ";
  for (const auto& pose : baseTrajectory.values_.at(ControlLevel::Position)) out << "[" << pose.getRotation() << "], ";
  out << std::endl;
  out << "Orientations (yaw, pitch, roll) [deg]: : ";
  for (const auto& pose : baseTrajectory.values_.at(ControlLevel::Position)) out << "[" << 180.0 / M_PI * EulerAnglesZyx(pose.getRotation()).getUnique().vector().transpose() << "], ";
  out << std::endl;
  return out;
}

} /* namespace */
