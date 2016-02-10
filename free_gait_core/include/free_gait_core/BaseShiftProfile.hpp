/*
 * BaseShiftProfile.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include <free_gait_core/base_motion/BaseMotionBase.hpp>
#include "free_gait_core/TypeDefs.hpp"
#include <string>

// Curves
#include "curves/CubicHermiteSE3Curve.hpp"

namespace free_gait {

class BaseShiftProfile : public BaseShiftTrajectoryBase
{
 public:
  typedef typename curves::CubicHermiteSE3Curve::ValueType ValueType;
  typedef typename curves::Time Time;

  BaseShiftProfile();
  virtual ~BaseShiftProfile();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  std::unique_ptr<BaseShiftTrajectoryBase> clone() const;

  /*!
   * Update the profile with the base start pose.
   * Do this to avoid jumps of the base.
   * @param startPose the start pose of the base in the frameId_ frame.
   * @return true if successful, false otherwise.
   */
  virtual bool updateStartPose(const Pose& startPose);

  /*!
   * Evaluate the base shift pose at a given time.
   * @param time the time value.
   * @return the pose of the base shift trajectory.
   */
  virtual const Pose evaluate(const double time);

  /*!
   * Returns the total duration of the trajectory.
   * @return the duration.
   */
  double getDuration() const;

  void setDuration(double duration);

  bool hasTarget() const;

  double getHeight() const;

  void setHeight(double height);

  const Pose& getTarget() const;

  void setTarget(const Pose& target);

  const std::string& getProfileType() const;

  void setProfileType(const std::string& profileType);

  friend std::ostream& operator << (std::ostream& out, const BaseShiftProfile& baseShiftProfile);

 protected:
  Pose start_; // In world frame.
  bool hasTarget_;
  Pose target_; // In world frame.
  double height_; // In control frame.
  double duration_;
  std::string profileType_;

 private:

  /*!
   * Computes the internal trajectory based on the profile type.
   */
  bool computeTrajectory();

  //! Base trajectory.
  curves::CubicHermiteSE3Curve trajectory_;

  //! If trajectory is updated.
  bool trajectoryUpdated_;
};

} /* namespace */
