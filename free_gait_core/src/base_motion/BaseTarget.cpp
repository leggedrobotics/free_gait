/*
 * BaseTarget.cpp
 *
 *  Created on: Mar 22, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/base_motion/BaseTarget.hpp"
#include <free_gait_core/base_motion/BaseMotionBase.hpp>

#include <math.h>

namespace free_gait {

BaseTarget::BaseTarget()
    : BaseMotionBase(BaseMotionBase::Type::Target),
      ignoreTimingOfLegMotion_(false),
      averageLinearVelocity_(0.0),
      averageAngularVelocity_(0.0),
      minimumDuration_(0.0),
      duration_(0.0),
      isComputed_(false),
      controlSetup_ { {ControlLevel::Position, true}, {ControlLevel::Velocity, false},
                      {ControlLevel::Acceleration, false}, {ControlLevel::Effort, false} }
{
}

BaseTarget::BaseTarget(const Pose& targetPose)
    : BaseMotionBase(BaseMotionBase::Type::Target),
      ignoreTimingOfLegMotion_(false),
      averageLinearVelocity_(0.0),
      averageAngularVelocity_(0.0),
      minimumDuration_(0.0),
      duration_(0.0),
      isComputed_(false),
      controlSetup_ { {ControlLevel::Position, true}, {ControlLevel::Velocity, false},
                      {ControlLevel::Acceleration, false}, {ControlLevel::Effort, false} }
{
  target_ = targetPose;
}

BaseTarget::~BaseTarget()
{
}

std::unique_ptr<BaseMotionBase> BaseTarget::clone() const
{
  std::unique_ptr<BaseMotionBase> pointer(new BaseTarget(*this));
  return pointer;
}

const ControlSetup BaseTarget::getControlSetup() const
{
  return controlSetup_;
}

void BaseTarget::updateStartPose(const Pose& startPose)
{
  isComputed_ = false;
  start_ = startPose;
}

bool BaseTarget::prepareComputation(const State& state, const Step& step, const StepQueue& queue, const AdapterBase& adapter)
{
  computeDuration(step, adapter);
  return isComputed_ = computeTrajectory();
}

bool BaseTarget::needsComputation() const
{
  return false;
}

bool BaseTarget::isComputed() const
{
  return isComputed_;
}

void BaseTarget::reset()
{
  start_.setIdentity();
  duration_ = 0.0;
  isComputed_ = false;
}

double BaseTarget::getDuration() const
{
  return duration_;
}

const std::string& BaseTarget::getFrameId(const ControlLevel& controlLevel) const
{
  return frameId_;
}

Pose BaseTarget::evaluatePose(const double time) const
{
  double timeInRange = time <= duration_ ? time : duration_;
  Pose pose;
  trajectory_.evaluate(pose, timeInRange);
  return pose;
}

Twist BaseTarget::evaluateTwist(const double time) const
{
  double timeInRange = time <= duration_ ? time : duration_;
  curves::CubicHermiteSE3Curve::DerivativeType derivative;
  trajectory_.evaluateDerivative(derivative, timeInRange, 1);
  Twist twist(derivative.getTranslationalVelocity().vector(),
              derivative.getRotationalVelocity().vector());
  return twist;
}

void BaseTarget::computeDuration(const Step& step, const AdapterBase& adapter)
{
  if (!step.hasLegMotion() || ignoreTimingOfLegMotion_) {
    double distance = (target_.getPosition() - start_.getPosition()).norm();
    double translationDuration = distance / averageLinearVelocity_;
    double angle = fabs(target_.getRotation().getDisparityAngle(start_.getRotation()));
    double rotationDuration = angle / averageAngularVelocity_;
    duration_ = translationDuration > rotationDuration ? translationDuration : rotationDuration;
  } else {
    for (const auto& limb : adapter.getLimbs()) {
      if (step.hasLegMotion(limb)) {
        if (!step.getLegMotion(limb).isIgnoreForPoseAdaptation()) {
          if (duration_ < step.getLegMotion(limb).getDuration()) duration_ = step.getLegMotion(limb).getDuration();
        }
      }
    }
  }

  duration_ = duration_ < minimumDuration_ ? minimumDuration_ : duration_;
}

bool BaseTarget::computeTrajectory()
{
  std::vector<Time> times;
  std::vector<ValueType> values;

  times.push_back(0.0);
  values.push_back(start_);

  times.push_back(duration_);
  values.push_back(target_);

  trajectory_.fitCurve(times, values);

  // Curves implementation provides velocities.
  controlSetup_[ControlLevel::Velocity] = true;

  return true;
}

std::ostream& operator<<(std::ostream& out, const BaseTarget& baseTarget)
{
  out << "Frame: " << baseTarget.frameId_ << std::endl;
  out << "Start Position: " << baseTarget.start_.getPosition() << std::endl;
  out << "Start Orientation: " << baseTarget.start_.getRotation() << std::endl;
  out << "Start Orientation (yaw, pitch, roll) [deg]: " << 180.0 / M_PI * EulerAnglesZyx(baseTarget.start_.getRotation()).getUnique().vector().transpose() << std::endl;
  out << "Target Position: " << baseTarget.target_.getPosition() << std::endl;
  out << "Target Orientation: " << baseTarget.target_.getRotation() << std::endl;
  out << "Target Orientation (yaw, pitch, roll) [deg]: " << 180.0 / M_PI * EulerAnglesZyx(baseTarget.target_.getRotation()).getUnique().vector().transpose() << std::endl;
  out << "Duration: " << baseTarget.duration_ << std::endl;
  return out;
}

} /* namespace */
