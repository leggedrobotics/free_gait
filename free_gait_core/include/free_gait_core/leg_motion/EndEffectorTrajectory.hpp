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
#include <curves/PolynomialSplineVectorSpaceCurve.hpp>

// STD
#include <string>
#include <memory>

namespace free_gait {

class EndEffectorTrajectory : public EndEffectorMotionBase
{
 public:
  typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType ValueType;
  typedef typename curves::PolynomialSplineQuinticVector3Curve::DerivativeType DerivativeType;
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

  const LinearVelocity evaluateVelocity(const double time) const;

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

  //! Foot trajectory.
  curves::PolynomialSplineQuinticVector3Curve trajectory_;

  //! If trajectory is updated.
  bool isComputed_;
};

} /* namespace */
