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
#include <free_gait_core/TypePrints.hpp>
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
    LegMode,
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

  LegMotionBase(const LegMotionBase& other);
  LegMotionBase& operator=(const LegMotionBase& other);

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

  virtual bool prepareComputation(const State& state, const Step& step, const AdapterBase& adapter);
  virtual bool needsComputation() const;
  virtual bool compute();
  virtual bool isComputed() const;
  virtual void reset();

  /*!
   * Returns the total duration of the motion.
   * @return the duration.
   */
  virtual double getDuration() const;

  bool hasSurfaceNormal() const;
  const Vector& getSurfaceNormal() const;
  void setSurfaceNormal(const Vector& surfaceNormal);

  bool hasFrictionCoefficient() const;
  double getFrictionCoefficient() const;
  void setFrictionCoefficient(double frictionCoefficient);

  virtual bool isIgnoreContact() const;

  virtual bool isIgnoreForPoseAdaptation() const;

  virtual bool hasContactAtStart() const;

  /*!
   * Print the contents to console for debugging.
   * @param out the output stream.
   * @param swingTrajectory the swing trajectory to debug.
   * @return the resulting output stream.
   */
  friend std::ostream& operator<< (std::ostream& out, const LegMotionBase& legMotion);
  friend class StepCompleter;

 protected:
  /*!
   * Returns a desired time to fit within the start and end time of the motion.
   * @param time the desired time.
   * @return the time mapped within the motion duration.
   */
  double mapTimeWithinDuration(const double time) const;

  LimbEnum limb_;
  std::unique_ptr<Vector> surfaceNormal_;
  std::unique_ptr<double> frictionCoefficient_;
  bool hasContactAtStart_;

 private:
  //! Type of the leg motion.
  Type type_;
};

std::ostream& operator<< (std::ostream& out, const LegMotionBase::Type& type);

} /* namespace */
