/*
 * LegMotionBase.hpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include <free_gait_core/TypeDefs.hpp>

// STD
#include <string>
#include <memory>

namespace free_gait {

/*!
 * Base class for a generic swing leg motion.
 */
class LegMotionBase
{
 public:

  enum class Type
  {
    Footstep,
    EndEffectorTarget,
    EndEffectorTrajectory,
    JointTarget,
    JointTrajectory
  };

  enum class TrajectoryType
  {
    EndEffector,
    Joints
  };

  /*!
   * Constructor.
   */
  LegMotionBase(LegMotionBase::Type type);

  /*!
   * Destructor.
   */
  virtual ~LegMotionBase();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  virtual std::unique_ptr<LegMotionBase> clone() const;

  /*!
   * Returns the type of the leg motion trajectory.
   * @return the type of the leg motion trajectory.
   */
  LegMotionBase::Type getType() const;
  virtual LegMotionBase::TrajectoryType getTrajectoryType() const;
  virtual const ControlSetup getControlSetup() const;

  /*!
   * Returns the total duration of the motion.
   * @return the duration.
   */
  virtual double getDuration() const;

  const Vector& getSurfaceNormal() const;

  bool isIgnoreContact() const;

  bool isIgnoreForPoseAdaptation() const;

  /*!
   * Print the contents to console for debugging.
   * @param out the output stream.
   * @param swingTrajectory the swing trajectory to debug.
   * @return the resulting output stream.
   */
  friend std::ostream& operator << (std::ostream& out, const LegMotionBase& legMotion);

 private:

  //! Type of the leg motion.
  Type type_;
};

} /* namespace */
