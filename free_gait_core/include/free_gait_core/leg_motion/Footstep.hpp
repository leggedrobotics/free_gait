/*
 * Footstep.hpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/leg_motion/EndEffectorMotionBase.hpp"

// Curves
#include <curves/PolynomialSplineVectorSpaceCurve.hpp>

// STD
#include <string>
#include <memory>

namespace free_gait {

class Footstep : public EndEffectorMotionBase
{
 public:
  typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType ValueType;
  typedef typename curves::Time Time;

  Footstep();
  virtual ~Footstep();

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
  void updateStartPosition(const Position& startPosition);

  bool compute(const State& state, const Step& step, const AdapterBase& adapter);

  /*!
   * Evaluate the swing foot position at a given swing phase value.
   * @param phase the swing phase value.
   * @return the position of the foot on the swing trajectory.
   */
  const Position evaluatePosition(const double phase) const;

  /*!
   * Returns the total duration of the trajectory.
   * @return the duration.
   */
  double getDuration() const;

  /*!
   * Return the target (end position) of the swing profile.
   * @return the target.
   */
  virtual const Position getTargetPosition() const;

  const std::string& getFrameId() const;

  const Vector& getSurfaceNormal() const;

  bool isIgnoreContact() const;

  bool isIgnoreForPoseAdaptation() const;

  friend std::ostream& operator << (std::ostream& out, const Footstep& footstep);
  friend class StepCompleter;
  friend class StepRosConverter;

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
  bool ignoreContact_;
  bool ignoreForPoseAdaptation_;

  //! Foot trajectory.
  curves::PolynomialSplineQuinticVector3Curve trajectory_;

  //! If trajectory is updated.
  bool updated_;
};

} /* namespace */
