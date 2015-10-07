/*
 * BaseShiftSplineTrajectory.hpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include <free_gait_core/BaseMotionBase.hpp>
#include "curves/CubicHermiteSE3Curve.hpp"

// STD
#include <memory>

namespace free_gait {

/*!
 * Implementation of a base shift trajectory as polynomial spline.
 */
class BaseShiftSplineTrajectory : public BaseShiftTrajectoryBase
{
 public:
  /*!
   * Constructor.
   */
  BaseShiftSplineTrajectory();

  /*!
   * Destructor.
   */
  virtual ~BaseShiftSplineTrajectory();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  virtual std::unique_ptr<BaseShiftTrajectoryBase> clone() const;

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
   * @return the pose of the base.
   */
  const Pose evaluate(const double time);

  /*!
   * Returns the total duration of the trajectory.
   * @return the duration.
   */
  double getDuration() const;

  /*!
   * Print the contents to console for debugging.
   * @param out the output stream.
   * @param baseShiftTrajectory the base shift trajectory to debug.
   * @return the resulting output stream.
   */
  friend std::ostream& operator << (std::ostream& out, const BaseShiftSplineTrajectory& baseShiftSplineTrajectory);

 protected:

  //! Base trajectory.
  curves::CubicHermiteSE3Curve trajectory_;
};

} /* namespace */
