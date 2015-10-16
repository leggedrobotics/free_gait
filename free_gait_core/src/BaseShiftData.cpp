/*
 * BaseShiftData.cpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/BaseShiftData.hpp"
#include "free_gait_core/BaseShiftProfile.hpp"
#include "free_gait_core/BaseShiftSplineTrajectory.hpp"

#include <memory>

namespace free_gait {

BaseShiftData::BaseShiftData()
{
}

BaseShiftData::BaseShiftData(const BaseShiftData& other) :
    name_(other.name_),
    ignore_(other.ignore_),
    trajectory_(other.trajectory_->clone())
{
}

BaseShiftData::~BaseShiftData()
{
}

const std::string& BaseShiftData::getName() const
{
  return name_;
}

void BaseShiftData::setName(const std::string& name)
{
  name_ = name;
}

bool BaseShiftData::isIgnore() const
{
  return ignore_;
}

void BaseShiftData::setIgnore(bool ignore)
{
  ignore_ = ignore;
}

const BaseShiftTrajectoryBase& BaseShiftData::getTrajectory() const
{
  return *trajectory_;
}

std::shared_ptr<BaseShiftTrajectoryBase> BaseShiftData::getTrajectory()
{
  return trajectory_;
}

void BaseShiftData::setTrajectory(const BaseShiftTrajectoryBase& trajectory)
{
  trajectory_ = trajectory.clone();
}

std::ostream& operator<<(std::ostream& out, const BaseShiftData& baseShiftData)
{
  out << "Name: " << baseShiftData.name_ << std::endl;
  if (baseShiftData.getTrajectory().getType() == BaseShiftTrajectoryType::Profile) {
    const auto& trajectory = std::static_pointer_cast<BaseShiftProfile>(baseShiftData.trajectory_);
    out << *trajectory << std::endl;
  } else if (baseShiftData.getTrajectory().getType() == BaseShiftTrajectoryType::Trajectory) {
    const auto& trajectory = std::static_pointer_cast<BaseShiftSplineTrajectory>(baseShiftData.trajectory_);
    out << *trajectory << std::endl;
  }
    return out;
}

} /* namespace */
