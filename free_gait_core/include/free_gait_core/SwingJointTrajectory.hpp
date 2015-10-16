/*
 * SwingJointTrajectory.hpp
 *
 *  Created on: Sep 2, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/SwingTrajectoryBase.hpp"

// Curves
#include <curves/PolynomialSplineScalarCurve.hpp>

// Loco // TODO Replace with robot model
#include <loco/common/LegBase.hpp>

// STD
#include <memory>

namespace free_gait {

// TODO Make this class nice based on new containers for joints.

/*!
 * Implementation of a joint trajectory as polynomial spline.
 */
class SwingJointTrajectory : public SwingTrajectoryBase
{
 public:
  typedef typename curves::PolynomialSplineQuinticScalarCurve::ValueType ValueType;
  typedef typename curves::Time Time;

  /*!
   * Constructor.
   */
  SwingJointTrajectory();

  /*!
   * Destructor.
   */
  virtual ~SwingJointTrajectory();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  virtual std::unique_ptr<SwingTrajectoryBase> clone() const;

  /*!
   * Update the trajectory with the joint start positions.
   * Do this to avoid jumps of the swing leg.
   * @param startPosition the start position of the joints frame.
   * @return true if successful, false otherwise.
   */
  // TODO This is a nasty overload, that shoudn't work if we'd use type safe data.
  bool updateStartPosition(const loco::LegBase::JointPositions& startPositions);

  /*!
   * Evaluate the swing joint positions at a given swing phase value.
   * @param phase the swing phase value.
   * @return the joint positions.
   */
  // TODO This is a nasty overload, that shoudn't work if we'd use type safe data.
//  const Position evaluate(const double phase);
  const loco::LegBase::JointPositions evaluate(const double phase);

  /*!
   * Returns the total duration of the trajectory.
   * @return the duration.
   */
  double getDuration() const;

  /*!
   * Return the target (end position) of the swing joint trajectories.
   * @return the target.
   */
  // TODO This is a nasty overload, that shoudn't work if we'd use type safe data.
//  const Position getTarget() const;
  const loco::LegBase::JointPositions getTarget() const;

  /*!
   * Print the contents to console for debugging.
   * @param out the output stream.
   * @param baseShiftTrajectory the base shift trajectory to debug.
   * @return the resulting output stream.
   */
  friend std::ostream& operator << (std::ostream& out, const SwingJointTrajectory& swingJointTrajectory);

 protected:

  //! Knots.
  std::vector<Time> times_;
  std::vector<std::vector<ValueType>> values_;

 private:

  /*!
   * Computes the internal trajectory based on the knots.
   */
  bool computeTrajectory();

  //! If trajectory is updated.
  bool trajectoryUpdated_;

  //! Joint trajectories, updated based on knots.
  std::vector<curves::PolynomialSplineQuinticScalarCurve> trajectories_;
};

} /* namespace */
