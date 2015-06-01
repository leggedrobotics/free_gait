/*
 * BaseShiftSplineTrajectory.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/BaseShiftSplineTrajectory.hpp>

namespace free_gait {

BaseShiftSplineTrajectory::BaseShiftSplineTrajectory() : BaseShiftTrajectoryBase()
{
}

BaseShiftSplineTrajectory::~BaseShiftSplineTrajectory()
{
}

std::unique_ptr<BaseShiftTrajectoryBase> BaseShiftSplineTrajectory::clone() const
{
  std::unique_ptr<BaseShiftTrajectoryBase> pointer(new BaseShiftSplineTrajectory(*this));
  return pointer;
}

bool BaseShiftSplineTrajectory::updateStartPose(const Pose& startPose)
{
  return true;
}

const Pose BaseShiftSplineTrajectory::evaluate(const double time)
{
  return Pose(trajectory_.evaluate(time));
}

double BaseShiftSplineTrajectory::getDuration() const
{
  return trajectory_.getMaxTime() - trajectory_.getMinTime();
}

std::ostream& operator<<(std::ostream& out, const BaseShiftSplineTrajectory& baseShiftSplineTrajectory)
{
  // TODO.
  return out;
}

} /* namespace loco */
