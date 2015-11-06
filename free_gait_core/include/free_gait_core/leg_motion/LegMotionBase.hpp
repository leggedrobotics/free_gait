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
#include <free_gait_core/step/Step.hpp>
#include <free_gait_core/executor/State.hpp>
#include <free_gait_core/executor/AdapterBase.hpp>

// STD
#include <string>
#include <memory>

namespace free_gait {

class State;
class Step;
class AdapterBase;

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
  LegMotionBase(LegMotionBase::Type type, LimbEnum limb);

  /*!
   * Destructor.
   */
  virtual ~LegMotionBase();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  virtual std::unique_ptr<LegMotionBase> clone() const;

  LimbEnum getLimb() const;

  /*!
   * Returns the type of the leg motion trajectory.
   * @return the type of the leg motion trajectory.
   */
  LegMotionBase::Type getType() const;
  virtual LegMotionBase::TrajectoryType getTrajectoryType() const;

  virtual const ControlSetup getControlSetup() const;

  virtual bool compute(const State& state, const Step& step, const AdapterBase& adapter);

  /*!
   * Returns the total duration of the motion.
   * @return the duration.
   */
  virtual double getDuration() const;

  virtual const Vector& getSurfaceNormal() const;

  virtual bool isIgnoreContact() const;

  virtual bool isIgnoreForPoseAdaptation() const;

  /*!
   * Print the contents to console for debugging.
   * @param out the output stream.
   * @param swingTrajectory the swing trajectory to debug.
   * @return the resulting output stream.
   */
  friend std::ostream& operator<< (std::ostream& out, const LegMotionBase& legMotion);

 protected:
  LimbEnum limb_;

 private:
  //! Type of the leg motion.
  Type type_;
};

std::ostream& operator<< (std::ostream& out, const LegMotionBase::Type& type);

} /* namespace */
