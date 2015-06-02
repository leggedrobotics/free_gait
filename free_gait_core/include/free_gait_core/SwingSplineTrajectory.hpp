/*
 * SwingSplineTrajectory.hpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/SwingTrajectoryBase.hpp"

// Curves
#include <curves/PolynomialSplineVectorSpaceCurve.hpp>

// STD
#include <memory>

namespace free_gait {

/*!
 * Implementation of a foot swing trajectory as polynomial spline.
 */
class SwingSplineTrajectory : public SwingTrajectoryBase
{
 public:
  /*!
   * Constructor.
   */
  SwingSplineTrajectory();

  /*!
   * Destructor.
   */
  virtual ~SwingSplineTrajectory();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  virtual std::unique_ptr<SwingTrajectoryBase> clone() const;

  /*!
   * Update the trajectory with the foot start position.
   * Do this to avoid jumps of the swing leg.
   * @param startPosition the start position of the foot in the trajectoryFrameId_ frame.
   * @return true if successful, false otherwise.
   */
  bool updateStartPosition(const Position& startPosition) ;

  /*!
   * Evaluate the swing foot position at a given swing phase value.
   * @param phase the swing phase value.
   * @return the position of the foot on the swing trajectory.
   */
  const Position evaluate(const double phase);

  /*!
   * Returns the total duration of the trajectory.
   * @return the duration.
   */
  double getDuration() const;

  /*!
   * Return the target (end position) of the swing trajectory.
   * @return the target.
   */
  const Position getTarget() const;

  /*!
   * Print the contents to console for debugging.
   * @param out the output stream.
   * @param baseShiftTrajectory the base shift trajectory to debug.
   * @return the resulting output stream.
   */
  friend std::ostream& operator << (std::ostream& out, const SwingSplineTrajectory& swingSplineTrajectory);

 protected:

  //! Foot trajectory.
  curves::PolynomialSplineQuinticVector3Curve trajectory_;
};

} /* namespace loco */
