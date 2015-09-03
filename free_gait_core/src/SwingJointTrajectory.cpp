/*
 * SwingJointTrajectory.cpp
 *
 *  Created on: Sep 2, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/SwingJointTrajectory.hpp>

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

bool SwingJointTrajectory::updateStartPosition(const Position& startPosition)
{
//  if (times_[0] == 0.0) {
//    values_[0] = startPosition.vector();
//  } else {
//    times_.insert(times_.begin(), 0.0);
//    values_.insert(values_.begin(), startPosition.vector());
//  }
//  trajectoryUpdated_ = false;
//  return computeTrajectory();
}

const Position SwingJointTrajectory::evaluate(const double phase)
{
//  if (!trajectoryUpdated_) computeTrajectory();
//  const double time = phase * getDuration();
//  return Position(trajectory_.evaluate(time));
}

double SwingJointTrajectory::getDuration() const
{
  //  return trajectory_.getMaxTime() - trajectory_.getMinTime();
  // We assume the swing spline trajectory always starts at time 0.0.
  return times_.back();
}

const Position SwingJointTrajectory::getTarget() const
{
//  return Position(trajectory_.evaluate(trajectory_.getMaxTime()));
}


bool SwingJointTrajectory::computeTrajectory()
{
  trajectory_.fitCurve(times_, values_);
  trajectoryUpdated_ = true;
  return true;
}

std::ostream& operator<<(std::ostream& out, const SwingJointTrajectory& swingJointTrajectory)
{
  // TODO.
  return out;
}

} /* namespace */
