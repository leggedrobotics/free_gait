/*
 * JointTrajectory.hpp
 *
 *  Created on: Nov 8, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/leg_motion/JointMotionBase.hpp"
#include <free_gait_core/TypeDefs.hpp>

// Curves
#include <curves/PolynomialSplineScalarCurve.hpp>

namespace free_gait {

class JointTrajectory : public JointMotionBase
{
 public:
  typedef typename curves::PolynomialSplineQuinticScalarCurve::ValueType ValueType;
  typedef typename curves::Time Time;

  /*!
   * Constructor.
   */
  JointTrajectory(LimbEnum limb);

  /*!
   * Destructor.
   */
  ~JointTrajectory();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  std::unique_ptr<LegMotionBase> clone() const;

  const ControlSetup getControlSetup() const;

  /*!
   * Update the trajectory with the foot start position.
   * Do this to avoid jumps of the swing leg.
   * @param footStartPosition the start position of the foot in the trajectoryFrameId_ frame.
   * @return true if successful, false otherwise.
   */
  void updateStartPosition(const JointPositions& startPosition);
  void updateStartVelocity(const JointVelocities& startVelocity);
  void updateStartAcceleration(const JointAccelerations& startAcceleration);
  void updateStartEfforts(const JointEfforts& startEffort);

  bool compute(const State& state, const Step& step, const AdapterBase& adapter);

  /*!
   * Returns the total duration of the motion.
   * @return the duration.
   */
  double getDuration() const;

  /*!
   * Evaluate the swing foot position at a given swing phase value.
   * @param phase the swing phase value.
   * @return the position of the foot on the swing trajectory.
   */
  const JointPositions evaluatePosition(const double time) const;
  const JointVelocities evaluateVelocity(const double time) const;
  const JointAccelerations evaluateAcceleration(const double time) const;
  const JointEfforts evaluateEffort(const double time) const;

  bool isIgnoreContact() const;

  /*!
   * Print the contents to console for debugging.
   * @param out the output stream.
   * @param swingTrajectory the swing trajectory to debug.
   * @return the resulting output stream.
   */
  friend std::ostream& operator << (std::ostream& out, const JointTrajectory& legMotion);
  friend class StepCompleter;
  friend class StepRosConverter;

 private:
  bool computed_;
  bool ignoreContact_;
  ControlSetup controlSetup_;

  //! Knots.
  std::unordered_map<ControlLevel, std::vector<Time>, EnumClassHash> times_;
  std::unordered_map<ControlLevel, std::vector<std::vector<ValueType>>, EnumClassHash> values_;

  double duration_;

  //! Joint trajectories, updated based on knots.
  std::unordered_map<ControlLevel, std::vector<curves::PolynomialSplineQuinticScalarCurve>, EnumClassHash> trajectories_;
};

} /* namespace */
