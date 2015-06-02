/*
 * SwingProfile.hpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/SwingTrajectoryBase.hpp"

// Curves
#include <curves/PolynomialSplineVectorSpaceCurve.hpp>

// STD
#include <string>
#include <memory>

namespace free_gait {

class SwingProfile : public SwingTrajectoryBase
{
 public:
  typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType ValueType;
  typedef typename curves::Time Time;

  SwingProfile();
  virtual ~SwingProfile();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  std::unique_ptr<SwingTrajectoryBase> clone() const;

  /*!
   * Update the trajectory with the foot start position.
   * Do this to avoid jumps of the swing leg.
   * @param startPosition the start position of the foot in the trajectoryFrameId_ frame.
   * @return true if successful, false otherwise.
   */
  virtual bool updateStartPosition(const Position& startPosition);

  /*!
   * Evaluate the swing foot position at a given swing phase value.
   * @param phase the swing phase value.
   * @return the position of the foot on the swing trajectory.
   */
  virtual const Position evaluate(const double phase);

  /*!
   * Returns the total duration of the trajectory.
   * @return the duration.
   */
  double getDuration() const;

  void setDuration(double duration);

  double getHeight() const;

  void setHeight(double height);

  /*!
   * Return the target (end position) of the swing profile.
   * @return the target.
   */
  const Position getTarget() const;

  void setTarget(const Position& target);

  const std::string& getType() const;

  void setType(const std::string& type);

  friend std::ostream& operator << (std::ostream& out, const SwingProfile& swingProfile);

 protected:
  Position start_;
  Position target_;
  double height_;
  double duration_;
  std::string type_;

 private:
  /*!
   * Computes the internal trajectory based on the profile type.
   */
  bool computeTrajectory();

  void generateTriangleKnots(std::vector<Time>& times, std::vector<ValueType>& values) const;

  void generateSquareKnots(std::vector<Time>& times, std::vector<ValueType>& values) const;

  //! Foot trajectory.
  curves::PolynomialSplineQuinticVector3Curve trajectory_;

  //! If trajectory is updated.
  bool trajectoryUpdated_;
};

} /* namespace loco */
