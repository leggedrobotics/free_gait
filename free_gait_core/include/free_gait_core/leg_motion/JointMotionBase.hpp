/*
 * JointMotionBase.hpp
 *
 *  Created on: Oct 21, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/leg_motion/LegMotionBase.hpp"
#include <free_gait_core/TypeDefs.hpp>

namespace free_gait {

/*!
 * Base class for end effector motion.
 */
class JointMotionBase : public LegMotionBase
{
 public:

  /*!
   * Constructor.
   */
  JointMotionBase(LegMotionBase::Type type, LimbEnum limb);

  /*!
   * Destructor.
   */
  virtual ~JointMotionBase();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  virtual std::unique_ptr<LegMotionBase> clone() const;

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
  virtual void updateStartPosition(const JointPositions& startPosition);
  virtual void updateStartVelocity(const JointVelocities& startVelocity);
  virtual void updateStartAcceleration(const JointAccelerations& startAcceleration);
  virtual void updateStartEfforts(const JointEfforts& startEffort);

  /*!
   * Evaluate the swing foot position at a given swing phase value.
   * @param phase the swing phase value.
   * @return the position of the foot on the swing trajectory.
   */
  virtual const JointPositions evaluatePosition(const double time) const;
  virtual const JointVelocities evaluateVelocity(const double time) const;
  virtual const JointAccelerations evaluateAcceleration(const double time) const;
  virtual const JointEfforts evaluateEffort(const double time) const;

  bool isIgnoreForPoseAdaptation() const;

  /*!
   * Print the contents to console for debugging.
   * @param out the output stream.
   * @param swingTrajectory the swing trajectory to debug.
   * @return the resulting output stream.
   */
  friend std::ostream& operator << (std::ostream& out, const JointMotionBase& legMotion);
};

} /* namespace */
