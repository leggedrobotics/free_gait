/*
 * SwingProfile.cpp.cpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/BaseShiftProfile.hpp"

#include <math.h>

namespace free_gait {

BaseShiftProfile::BaseShiftProfile()
    : BaseShiftTrajectoryBase(),
      hasTarget_(false),
      height_(0.0),
      duration_(0.0),
      trajectoryUpdated_(false)
{
  // TODO Auto-generated constructor stub

}

BaseShiftProfile::~BaseShiftProfile()
{
  // TODO Auto-generated destructor stub
}

std::unique_ptr<BaseShiftTrajectoryBase> BaseShiftProfile::clone() const
{
  std::unique_ptr<BaseShiftTrajectoryBase> pointer(new BaseShiftProfile(*this));
  return pointer;
}

bool BaseShiftProfile::updateStartPose(const Pose& startPose)
{
  start_.getPosition() = startPose.getPosition();
  start_.getRotation() = startPose.getRotation().getUnique();
  trajectoryUpdated_ = false;
  return true;
}

const Pose BaseShiftProfile::evaluate(const double time)
{
  if (!trajectoryUpdated_) computeTrajectory();
  return Pose(trajectory_.evaluate(time));
}

double BaseShiftProfile::getDuration() const
{
  return duration_;
}

void BaseShiftProfile::setDuration(double duration)
{
  duration_ = duration;
}

bool BaseShiftProfile::hasTarget() const
{
  return hasTarget_;
}

double BaseShiftProfile::getHeight() const
{
  return height_;
}

void BaseShiftProfile::setHeight(double height)
{
  height_ = height;
}

const Pose& BaseShiftProfile::getTarget() const
{
  return target_;
}

void BaseShiftProfile::setTarget(const Pose& target)
{
  target_.getPosition() = target.getPosition();
  target_.getRotation() = target.getRotation().getUnique();
  hasTarget_ = true;
}

const std::string& BaseShiftProfile::getType() const
{
  return type_;
}

void BaseShiftProfile::setType(const std::string& type)
{
  type_ = type;
}

bool BaseShiftProfile::computeTrajectory()
{
  if (!hasTarget_) return false;

  std::vector<Time> times;
  std::vector<ValueType> values;

  times.push_back(0.0);
  values.push_back(start_);

  times.push_back(duration_);
  values.push_back(target_);

  trajectory_.fitCurve(times, values);
  trajectoryUpdated_ = true;
  return true;
}

std::ostream& operator<<(std::ostream& out, const BaseShiftProfile& baseShiftProfile)
{
  out << "Start Position: " << baseShiftProfile.start_.getPosition() << std::endl;
  out << "Start Orientation: " << baseShiftProfile.start_.getRotation() << std::endl;
  out << "Start Orientation (yaw, pitch, roll) [deg]: " << 180.0 / M_PI * EulerAnglesZyx(baseShiftProfile.start_.getRotation()).getUnique().vector().transpose() << std::endl;
  out << "Target Position: " << baseShiftProfile.target_.getPosition() << std::endl;
  out << "Target Orientation: " << baseShiftProfile.target_.getRotation() << std::endl;
  out << "Target Orientation (yaw, pitch, roll) [deg]: " << 180.0 / M_PI * EulerAnglesZyx(baseShiftProfile.target_.getRotation()).getUnique().vector().transpose() << std::endl;
  out << "Height: " << baseShiftProfile.height_ << std::endl;
  out << "Duration: " << baseShiftProfile.duration_ << std::endl;
  out << "Type: " << baseShiftProfile.type_;
  return out;
}

} /* namespace loco */
