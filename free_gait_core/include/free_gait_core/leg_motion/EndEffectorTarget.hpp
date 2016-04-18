/*
 * EndEffectorTarget.hpp
 *
 *  Created on: Apr 18, 2016
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

class EndEffectorTarget : public EndEffectorMotionBase
{
 public:
  typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType ValueType;
  typedef typename curves::PolynomialSplineQuinticVector3Curve::DerivativeType DerivativeType;
  typedef typename curves::Time Time;

  EndEffectorTarget(LimbEnum limb);
  virtual ~EndEffectorTarget();

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

  /*!
   * Return the target (end position) of the swing profile.
   * @return the target.
   */
  const Position getTargetPosition() const;

  const std::string& getFrameId(const ControlLevel& controlLevel) const;

  bool isIgnoreContact() const;

  bool isIgnoreForPoseAdaptation() const;

  friend std::ostream& operator << (std::ostream& out, const EndEffectorTarget& endEffectorTarget);
  friend class StepCompleter;
  friend class StepRosConverter;

 private:
  void computeDuration();
  bool computeTrajectory();

  bool ignoreContact_;
  bool ignoreForPoseAdaptation_;
  double minimumDuration_;

  ControlSetup controlSetup_;
  std::unordered_map<ControlLevel, std::string, EnumClassHash> frameIds_;
  std::unordered_map<ControlLevel, ValueType, EnumClassHash> start_;
  std::unordered_map<ControlLevel, ValueType, EnumClassHash> target_;
  double duration_;
  double averageVelocity_;

  //! End effector trajectory.
  curves::PolynomialSplineQuinticVector3Curve trajectory_;

  //! If trajectory is updated.
  bool isComputed_;
};

} /* namespace */
