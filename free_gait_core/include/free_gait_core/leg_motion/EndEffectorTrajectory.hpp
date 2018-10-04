/*
 * EndEffectorTrajectory.hpp
 *
 *  Created on: Mar 16, 2016
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

class EndEffectorTrajectory : public EndEffectorMotionBase
{
 public:
  typedef typename curves::CubicHermiteE3Curve::ValueType ValueType;
  typedef typename curves::CubicHermiteE3Curve::DerivativeType DerivativeType;
  typedef typename curves::Time Time;

  EndEffectorTrajectory(LimbEnum limb);
  virtual ~EndEffectorTrajectory();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  std::unique_ptr<LegMotionBase> clone() const;

  /*!
   * Set the entire trajectory at once.
   * Note: Make sure to define a consistent trajectory with same control
   * level definitions and same number of time/value pairs.
   * Note: Alternatively, the `setFrameId(...)` and `addPositionTrajectoryPoint(...)`
   * etc. methods can be used to add trajectory points individually.
   * @param frameIds the frame Ids for each used control level.
   * @param times the time vector.
   * @param values the value vector associated with the time vector.
   */
  void setTrajectory(
      const std::unordered_map<ControlLevel, std::string, EnumClassHash>& frameIds,
      const std::vector<Time>& times,
      const std::unordered_map<ControlLevel, std::vector<ValueType>, EnumClassHash>& values);
  void setFrameId(const ControlLevel& controlLevel, const std::string& frameId);
  bool addPositionTrajectoryPoint(const Time& time, const Position& position);

  //! Update the trajectory with the foot start position.
  void updateStartPosition(const Position& startPosition) override;

  //! Update the trajectory with the foot start velocity.
  void updateStartVelocity(const LinearVelocity& startVelocity) override;

  //! Update the trajectory with the end-effector start force.
  void updateStartEndEffectorForce(const Force& force) override;

  const Position getStartPosition() const;
  const LinearVelocity getStartVelocity() const;

  const ControlSetup getControlSetup() const;

  bool prepareComputation(const State& state, const Step& step, const AdapterBase& adapter);
  bool needsComputation() const;
  bool isComputed() const;
  void reset();

  //! Evaluate the swing foot position at a given swing phase value.
  const Position evaluatePosition(const double time) const override;

  //! Evaluate the swing foot velocity at a given swing phase value.
  const LinearVelocity evaluateVelocity(const double time) const override;

  //! Evaluate the swing foot acceleration at a given swing phase value.
  const LinearAcceleration evaluateAcceleration(const double time) const override;

  //! Evaluate the swing foot force (at end-effector) at a given swing phase value.
  const Force evaluateEndEffectorForce(const double time) const override;

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

  void setIgnoreContact(bool ignoreContact);
  bool isIgnoreContact() const;

  bool isIgnoreForPoseAdaptation() const;

  friend std::ostream& operator << (std::ostream& out, const EndEffectorTrajectory& endEffectorTrajectory);
  friend class StepCompleter;
  friend class StepRosConverter;
  friend class StepFrameConverter;

 private:
  bool ignoreContact_;
  bool ignoreForPoseAdaptation_;

  ControlSetup controlSetup_;

  //! Knots.
  std::unordered_map<ControlLevel, std::string, EnumClassHash> frameIds_;
  std::vector<Time> times_;
  std::unordered_map<ControlLevel, std::vector<ValueType>, EnumClassHash> values_;

  double duration_;

  //! End effector trajectory.
  curves::CubicHermiteE3Curve trajectory_;

  //! End effector trajectory for contact force.
  curves::CubicHermiteE3Curve trajectoryForce_;

  //! If trajectory is updated.
  bool isComputed_;
};

} /* namespace */
