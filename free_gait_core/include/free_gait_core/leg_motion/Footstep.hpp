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
#include <curves/CubicHermiteE3Curve.hpp>

// STD
#include <string>
#include <memory>

namespace free_gait {

class Footstep : public EndEffectorMotionBase
{
 public:
  typedef typename curves::CubicHermiteE3Curve::ValueType ValueType;
  typedef typename curves::CubicHermiteE3Curve::DerivativeType DerivativeType;
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

  bool compute(bool isSupportLeg);
  bool prepareComputation(const State& state, const Step& step, const AdapterBase& adapter);
  bool needsComputation() const;
  bool isComputed() const;

  /*!
   * Evaluate the swing foot position at a given swing phase value.
   * @param phase the swing phase value.
   * @return the position of the foot on the swing trajectory.
   */
  const Position evaluatePosition(const double time) const;

  const LinearVelocity evaluateVelocity(const double time) const;

  /*!
   * Returns the total duration of the trajectory.
   * @return the duration.
   */
  double getDuration() const;

  void setStartPosition(const std::string& frameId, const Position& start);
  const Position getStartPosition() const;
  void setTargetPosition(const std::string& frameId, const Position& target);
  const Position getTargetPosition() const;
  const std::string& getFrameId(const ControlLevel& controlLevel) const;

  void setProfileType(const std::string& profileType);
  const std::string& getProfileType() const;
  void setProfileHeight(const double profileHeight);
  double getProfileHeight() const;
  double getAverageVelocity() const;
  void setAverageVelocity(double averageVelocity);

  bool isIgnoreContact() const;

  bool isIgnoreForPoseAdaptation() const;

  friend std::ostream& operator << (std::ostream& out, const Footstep& footstep);

  friend class StepCompleter;
  friend class StepRosConverter;
  friend class StepFrameConverter;

 private:
  void generateStraightKnots(std::vector<ValueType>& values) const;
  void generateTriangleKnots(std::vector<ValueType>& values) const;
  void generateSquareKnots(std::vector<ValueType>& values) const;
  void generateTrapezoidKnots(std::vector<ValueType>& values) const;
  void computeTiming(const std::vector<ValueType>& values, std::vector<Time>& times) const;

  Position start_;
  Position target_;
  std::string frameId_;
  double profileHeight_;
  std::string profileType_;
  double averageVelocity_;
  bool ignoreContact_;
  bool ignoreForPoseAdaptation_;
  double liftOffSpeed_;
  double touchdownSpeed_;
  double duration_;
  double minimumDuration_;

  ControlSetup controlSetup_;

  //! Foot trajectory.
  curves::CubicHermiteE3Curve trajectory_;

  //! If trajectory is updated.
  bool isComputed_;
};

} /* namespace */
