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
    : useProfile_(true)
{
}

BaseShiftData::BaseShiftData(const BaseShiftData& other) :
    name_(other.name_),
    trajectory_(other.trajectory_->clone()),
    useProfile_(other.useProfile_)
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

bool BaseShiftData::isUsingProfile() const
{
  return useProfile_;
}

void BaseShiftData::setUseProfile(bool useProfile)
{
  useProfile_ = useProfile;
}

std::ostream& operator<<(std::ostream& out, const BaseShiftData& baseShiftData)
{
  out << "Name: " << baseShiftData.name_ << std::endl;
  if (baseShiftData.useProfile_) {
    const auto& trajectory = std::static_pointer_cast<BaseShiftProfile>(baseShiftData.trajectory_);
    out << *trajectory << std::endl;
  } else {
    const auto& trajectory = std::static_pointer_cast<BaseShiftSplineTrajectory>(baseShiftData.trajectory_);
    out << *trajectory << std::endl;
  }
    return out;
}

} /* namespace loco */
