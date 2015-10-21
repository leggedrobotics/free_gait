/*
 * BaseAuto.hpp
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

class BaseAuto : public BaseMotionBase
{
 public:
  typedef typename curves::CubicHermiteSE3Curve::ValueType ValueType;
  typedef typename curves::Time Time;

  BaseAuto();
  virtual ~BaseAuto();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  std::unique_ptr<BaseMotionBase> clone() const;

  bool isIgnore() const;
  void setIgnore(bool ignore);

  /*!
   * Update the profile with the base start pose.
   * Do this to avoid jumps of the base.
   * @param startPose the start pose of the base in the frameId_ frame.
   * @return true if successful, false otherwise.
   */
  void updateStartPose(const Pose& startPose);

  /*!
   * Evaluate the base shift pose at a given time.
   * @param time the time value.
   * @return the pose of the base shift trajectory.
   */
  Pose evaluatePose(const double time);

  /*!
   * Returns the total duration of the trajectory.
   * @return the duration.
   */
  double getDuration() const;

  double getAverageVelocity() const;

  void setAverageVelocity(double averageVelocity);

  double getHeight() const;

  void setHeight(double height);

  bool hasTarget() const;

  const Pose& getTarget() const;

  void setTarget(const Pose& target);

  friend std::ostream& operator << (std::ostream& out, const BaseAuto& baseAuto);

 private:

  /*!
   * Computes the internal trajectory based on the profile type.
   */
  bool computeTrajectory();

  bool ignore_;

  Pose start_; // In world frame.
  bool hasTarget_;
  double height_; // In control frame.
  Pose target_; // In world frame.
  double averageVelocity_;
  double duration_;

  //! Base trajectory.
  curves::CubicHermiteSE3Curve trajectory_;

  //! If trajectory is updated.
  bool trajectoryUpdated_;
};

} /* namespace */
