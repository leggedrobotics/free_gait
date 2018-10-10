/*
 * EndEffectorMotionBase.hpp
 *
 *  Created on: Oct 21, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/leg_motion/LegMotionBase.hpp"
#include <free_gait_core/TypeDefs.hpp>

// STD
#include <string>
#include <memory>

namespace free_gait {

/*!
 * Base class for end effector motion.
 */
class EndEffectorMotionBase : public LegMotionBase
{
 public:

  /*!
   * Constructor.
   */
  EndEffectorMotionBase(LegMotionBase::Type type, LimbEnum limb);

  /*!
   * Destructor.
   */
  virtual ~EndEffectorMotionBase();

  /*!
   * Returns the type of the leg motion trajectory.
   * @return the type of the leg motion trajectory.
   */
  LegMotionBase::TrajectoryType getTrajectoryType() const;

  /*!
   * Update the trajectory with the foot start position.
   * Do this to avoid jumps of the swing leg.
   * @param footStartPosition the start position of the foot in the trajectoryFrameId_ frame.
   * @return true if successful, false otherwise.
   */
  virtual void updateStartPosition(const Position& startPosition);
  virtual void updateStartVelocity(const LinearVelocity& startVelocity);
  virtual void updateStartAcceleration(const LinearAcceleration& startAcceleration);
  virtual void updateStartEndEffectorForce(const Force& startForce);

  /*!
   * Evaluate the swing foot position at a given swing phase value.
   * @param phase the swing phase value.
   * @return the position of the foot on the swing trajectory.
   */
  virtual const Position evaluatePosition(const double time) const;
  virtual const LinearVelocity evaluateVelocity(const double time) const;
  virtual const LinearAcceleration evaluateAcceleration(const double time) const;
  virtual const Force evaluateEndEffectorForce(const double time) const;

  /*!
   * Return the target (end position) of the swing trajectory.
   * @return the target.
   */
  virtual const Position getTargetPosition() const;
  virtual const LinearVelocity getTargetVelocity() const;

  /*!
   * Returns the frame id of the trajectory.
   * @return the frame id.
   */
  virtual const std::string& getFrameId(const ControlLevel& controlLevel) const;

  /*!
   * Print the contents to console for debugging.
   * @param out the output stream.
   * @param swingTrajectory the swing trajectory to debug.
   * @return the resulting output stream.
   */
  friend std::ostream& operator << (std::ostream& out, const LegMotionBase& legMotion);

  virtual const Vector& getImpedanceGain(const ImpedanceControl& impedanceControlId) const;
  virtual const std::string& getImpedanceGainFrameId(const ImpedanceControl& impedanceControlId) const;
  virtual double getFeedForwardFrictionNorm() const;
  virtual bool getIsImpedanceTrajectory() const noexcept;
};

} /* namespace */
