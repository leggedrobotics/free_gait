/*
 * FootTarget.hpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/leg_motion/LegMotionBase.hpp"

// Curves
#include <curves/PolynomialSplineVectorSpaceCurve.hpp>

// STD
#include <string>
#include <memory>

namespace free_gait {

class FootTarget : public LegMotionBase
{
 public:
  typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType ValueType;
  typedef typename curves::Time Time;

  FootTarget();
  virtual ~FootTarget();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  std::unique_ptr<LegMotionBase> clone() const;

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

  double getAverageVelocity() const;

  void setAverageVelocity(double averageVelocity);

  double getProfileHeight() const;

  void setProfileHeight(double profileHeight);

  /*!
   * Return the target (end position) of the swing profile.
   * @return the target.
   */
  const Position getTarget() const;

  void setTarget(const Position& target);

  const std::string& getFrameId() const;

  void setFrameId(const std::string& frameId);

  const std::string& getProfileType() const;

  void setProfileType(const std::string& type);

  const Vector& getSurfaceNormal() const;

  void setSurfaceNormal(const Vector& surfaceNormal);

  bool isNoTouchdown() const;

  void setNoTouchdown(bool noTouchdown);

  bool isIgnoreForPoseAdaptation() const;

  void setIgnoreForPoseAdaptation(bool ignoreForPoseAdaptation);

  friend std::ostream& operator << (std::ostream& out, const FootTarget& footTarget);

  friend class FootTargetCompleter;

 private:
  /*!
   * Computes the internal trajectory based on the profile type.
   */
  bool computeTrajectory();

  void generateStraightKnots(std::vector<Time>& times, std::vector<ValueType>& values) const;

  void generateTriangleKnots(std::vector<Time>& times, std::vector<ValueType>& values) const;

  void generateSquareKnots(std::vector<Time>& times, std::vector<ValueType>& values) const;

  Position start_;
  Position target_;
  std::string frameId_;
  double profileHeight_;
  double averageVelocity_;
  std::string profileType_;
  Vector surfaceNormal_;
  bool noTouchdown_;
  bool ignoreForPoseAdaptation_;

  //! Foot trajectory.
  curves::PolynomialSplineQuinticVector3Curve trajectory_;

  //! If trajectory is updated.
  bool trajectoryUpdated_;
};

} /* namespace */
