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
  typedef typename curves::PolynomialSplineQuinticVector3Curve::DerivativeType DerivativeType;
  typedef typename curves::Time Time;

  Footstep(LimbEnum limb);
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

  const ControlSetup getControlSetup() const;

  bool prepareComputation(const State& state, const Step& step, const AdapterBase& adapter);
  bool needsComputation() const;
  bool isComputed() const;

  /*!
   * Evaluate the swing foot position at a given swing phase value.
   * @param phase the swing phase value.
   * @return the position of the foot on the swing trajectory.
   */
  const Position evaluatePosition(const double time) const;

  /*!
   * Returns the total duration of the trajectory.
   * @return the duration.
   */
  double getDuration() const;

  void setTargetPosition(const Position target);
  const Position getTargetPosition() const;

  void setFrameId(const ControlLevel& controlLevel, const std::string& frameId);
  const std::string& getFrameId(const ControlLevel& controlLevel) const;

  bool isIgnoreContact() const;

  bool isIgnoreForPoseAdaptation() const;

  friend std::ostream& operator << (std::ostream& out, const Footstep& footstep);
  friend class StepCompleter;
  friend class StepRosConverter;

 private:
  void generateStraightKnots(std::vector<ValueType>& values) const;
  void generateTriangleKnots(std::vector<ValueType>& values) const;
  void generateSquareKnots(std::vector<ValueType>& values) const;
  void computeTiming(const std::vector<ValueType>& values, std::vector<Time>& times) const;
  void computeVelocities(const std::vector<Time>& times, std::vector<DerivativeType>& velocities,
                         std::vector<DerivativeType>& accelerations) const;

  Position start_;
  Position target_;
  std::string frameId_;
  double profileHeight_;
  double averageVelocity_;
  std::string profileType_;
  bool ignoreContact_;
  bool ignoreForPoseAdaptation_;
  double liftOffVelocity_;
  double touchdownVelocity_;
  double minimumDuration_;

  ControlSetup controlSetup_;

  //! Foot trajectory.
  curves::PolynomialSplineQuinticVector3Curve trajectory_;

  //! If trajectory is updated.
  bool isComputed_;
};

} /* namespace */
