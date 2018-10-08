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
#include <curves/CubicHermiteE3Curve.hpp>

// STD
#include <string>
#include <memory>

namespace free_gait {

class EndEffectorTarget : public EndEffectorMotionBase
{
 public:
  typedef typename curves::CubicHermiteE3Curve::ValueType ValueType;
  typedef typename curves::CubicHermiteE3Curve::DerivativeType DerivativeType;
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
  void updateStartPosition(const Position& startPosition) override;
  void updateStartVelocity(const LinearVelocity& startVelocity) override;
  void updateStartEndEffectorForce(const Force& startForce) override;

  const ControlSetup getControlSetup() const;

  bool prepareComputation(const State& state, const Step& step, const AdapterBase& adapter);
  bool needsComputation() const;
  bool isComputed() const;
  void reset();

  /*!
   * Evaluate the swing foot position at a given swing phase value.
   * @param phase the swing phase value.
   * @return the position of the foot on the swing trajectory.
   */
  const Position evaluatePosition(const double time) const override;
  const LinearVelocity evaluateVelocity(const double time) const override;
  const LinearAcceleration evaluateAcceleration(const double time) const override;
  const Force evaluateEndEffectorForce(const double time) const override;

  /*!
   * Returns the total duration of the trajectory.
   * @return the duration.
   */
  double getDuration() const;

  void setTargetPosition(const std::string& frameId, const Position& targetPosition);
  void setTargetVelocity(const std::string& frameId, const LinearVelocity& targetVelocity);

  /*!
   * Return the target (end position) of the swing profile.
   * @return the target.
   */
  const Position getTargetPosition() const;
  const LinearVelocity getTargetVelocity() const;

  const std::string& getFrameId(const ControlLevel& controlLevel) const;

  bool isIgnoreContact() const;

  bool isIgnoreForPoseAdaptation() const;

  void setAverageVelocity(const double averageVelocity);

  friend std::ostream& operator << (std::ostream& out, const EndEffectorTarget& endEffectorTarget);
  friend class StepCompleter;
  friend class StepRosConverter;

  const Vector& getImpedanceGain(const ImpedanceControl& impedanceControlId) const override;
  const std::string& getImpedanceGainFrameId(const ImpedanceControl& impedanceControlId) const override;
  double getFeedForwardFrictionNorm() const override;

 private:
  void computeDuration();
  bool computeTrajectory();

  bool ignoreContact_;
  bool ignoreForPoseAdaptation_;
  double minimumDuration_;

  ControlSetup controlSetup_;
  std::unordered_map<ControlLevel, std::string, EnumClassHash> frameIds_;
  Position startPosition_;
  Position targetPosition_;
  LinearVelocity startVelocity_;
  LinearVelocity targetVelocity_;
  Force startForce_;
  Force targetForce_;
  double duration_;
  double averageVelocity_;
  bool useAverageVelocity_;

  //! End effector trajectory.
  curves::CubicHermiteE3Curve trajectory_;

  //! If trajectory is updated.
  bool isComputed_;

  // Impedance Control gains.
  std::unordered_map<ImpedanceControl, Vector, EnumClassHash> impedanceGains_;
  std::unordered_map<ImpedanceControl, std::string, EnumClassHash> impedanceGainsFrameId_;
  double feedForwardFrictionNorm_;
};

} /* namespace */
