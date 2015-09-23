/*
 * SwingJointTrajectory.cpp
 *
 *  Created on: Sep 2, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/SwingJointTrajectory.hpp>

// Roco
#include <roco/log/log_messages.hpp>

namespace free_gait {

SwingJointTrajectory::SwingJointTrajectory()
    : SwingTrajectoryBase(SwingTrajectoryType::JointTrajectory),
      trajectoryUpdated_(false)
{
}

SwingJointTrajectory::~SwingJointTrajectory()
{
}

std::unique_ptr<SwingTrajectoryBase> SwingJointTrajectory::clone() const
{
  std::unique_ptr<SwingTrajectoryBase> pointer(new SwingJointTrajectory(*this));
  return pointer;
}

bool SwingJointTrajectory::updateStartPosition(const loco::LegBase::JointPositions& startPositions)
{
  if (times_[0] == 0.0) {
    for (size_t i = 0; i < values_.size(); ++i) {
      values_[i][0] = startPositions(i);
    }
  } else {
    times_.insert(times_.begin(), 0.0);
    for (size_t i = 0; i < values_.size(); ++i) {
      values_[i].insert(values_[i].begin(), startPositions(i));
    }
  }
  trajectoryUpdated_ = false;
  return computeTrajectory();
}

const loco::LegBase::JointPositions SwingJointTrajectory::evaluate(const double phase)
{
  if (!trajectoryUpdated_) computeTrajectory();
  const double time = phase * getDuration();
  loco::LegBase::JointPositions jointPositions;
  for (size_t i = 0; i < trajectories_.size(); ++i) {
    jointPositions(i) = trajectories_[i].evaluate(time);
  }
  return jointPositions;
}

double SwingJointTrajectory::getDuration() const
{
  //  return trajectory_.getMaxTime() - trajectory_.getMinTime();
  // We assume the swing spline trajectory always starts at time 0.0.
  return times_.back();
}

const loco::LegBase::JointPositions SwingJointTrajectory::getTarget() const
{
  loco::LegBase::JointPositions jointPositions;
  for (size_t i = 0; i < trajectories_.size(); ++i) {
    jointPositions(i) = trajectories_[i].evaluate(trajectories_[i].getMaxTime());
  }
  return jointPositions;
}

bool SwingJointTrajectory::computeTrajectory()
{
  trajectories_.resize(values_.size());
  for (size_t i = 0; i < values_.size(); ++i) {
    trajectories_[i].fitCurve(times_, values_[i]);
  }
  trajectoryUpdated_ = true;
  return true;
}

std::ostream& operator<<(std::ostream& out, const SwingJointTrajectory& swingJointTrajectory)
{
  // TODO.
  return out;
}

} /* namespace */
