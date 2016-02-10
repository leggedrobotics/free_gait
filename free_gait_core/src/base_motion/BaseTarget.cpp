/*
 * BaseAuto.cpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/base_motion/BaseAuto.hpp"
#include <free_gait_core/base_motion/BaseMotionBase.hpp>

#include <math.h>

namespace free_gait {

BaseTarget::BaseTarget()
    : BaseMotionBase(BaseMotionBase::Type::Auto),
      hasTarget_(false),
      averageVelocity_(0.0),
      duration_(0.0),
      trajectoryUpdated_(false)
{
}

BaseTarget::~BaseTarget()
{
}

std::unique_ptr<BaseMotionBase> BaseTarget::clone() const
{
  std::unique_ptr<BaseMotionBase> pointer(new BaseAuto(*this));
  return pointer;
}

bool BaseTarget::updateStartPose(const Pose& startPose)
{
  start_.getPosition() = startPose.getPosition();
  start_.getRotation() = startPose.getRotation().getUnique();
  trajectoryUpdated_ = false;
  return true;
}

const Pose BaseTarget::evaluate(const double time)
{
  if (!trajectoryUpdated_) computeTrajectory();
  double timeInRange = time <= duration_ ? time : duration_;
  return Pose(trajectory_.evaluate(timeInRange));
}

double BaseTarget::getDuration() const
{
  return duration_;
}

double BaseTarget::getAverageVelocity() const
{
  return averageVelocity_;
}

void BaseTarget::setAverageVelocity(double averageVelocity)
{
  averageVelocity_ = averageVelocity;
  trajectoryUpdated_ = false;
}

bool BaseTarget::hasTarget() const
{
  return hasTarget_;
}

double BaseTarget::getHeight() const
{
  return height_;
}

void BaseTarget::setHeight(double height)
{
  height_ = height;
}

const Pose& BaseTarget::getTarget() const
{
  return target_;
}

void BaseTarget::setTarget(const Pose& target)
{
  target_.getPosition() = target.getPosition();
  target_.getRotation() = target.getRotation().getUnique();
  hasTarget_ = true;
}

bool BaseTarget::computeTrajectory()
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

std::ostream& operator<<(std::ostream& out, const BaseTarget& baseAuto)
{
//  out << "Start Position: " << baseShiftProfile.start_.getPosition() << std::endl;
//  out << "Start Orientation: " << baseShiftProfile.start_.getRotation() << std::endl;
//  out << "Start Orientation (yaw, pitch, roll) [deg]: " << 180.0 / M_PI * EulerAnglesZyx(baseShiftProfile.start_.getRotation()).getUnique().vector().transpose() << std::endl;
//  out << "Target Position: " << baseShiftProfile.target_.getPosition() << std::endl;
//  out << "Target Orientation: " << baseShiftProfile.target_.getRotation() << std::endl;
//  out << "Target Orientation (yaw, pitch, roll) [deg]: " << 180.0 / M_PI * EulerAnglesZyx(baseShiftProfile.target_.getRotation()).getUnique().vector().transpose() << std::endl;
//  out << "Height: " << baseShiftProfile.height_ << std::endl;
//  out << "Duration: " << baseShiftProfile.duration_ << std::endl;
//  out << "Type: " << baseShiftProfile.profileType_;
  return out;
}

} /* namespace */
