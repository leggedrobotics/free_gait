/*
 * BaseTarget.hpp
 *
 *  Created on: Mar 22, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/base_motion/BaseMotionBase.hpp"
#include "free_gait_core/TypeDefs.hpp"

// Curves
#include "curves/CubicHermiteSE3Curve.hpp"

namespace free_gait {

class BaseTarget : public BaseMotionBase
{
 public:
  typedef typename curves::CubicHermiteSE3Curve::ValueType ValueType;
  typedef typename curves::Time Time;

  BaseTarget();
  BaseTarget(const Pose& targetPose);
  virtual ~BaseTarget();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  std::unique_ptr<BaseMotionBase> clone() const;

  const ControlSetup getControlSetup() const;

  /*!
   * Update the base motion with the base start pose.
   * Do this to avoid jumps of the base.
   * @param startPose the start pose of the base in the frameId_ frame.
   * @return true if successful, false otherwise.
   */
  virtual void updateStartPose(const Pose& startPose);

  /*!
   * Set the frame ID for base target pose.
   * @param frameId target frame ID.
   */
  void setFrameId(const std::string& frameId);

  bool prepareComputation(const State& state, const Step& step, const StepQueue& queue,
                          const AdapterBase& adapter);
  bool needsComputation() const;
  bool isComputed() const;
  void reset();

  /*!
   * Returns the total duration of the trajectory.
   * @return the duration.
   */
  double getDuration() const;

  /*!
   * Returns the frame id base motion.
   * @return the frame id.
   */
  const std::string& getFrameId(const ControlLevel& controlLevel) const;

  /*!
   * Evaluate the base motion pose at a given time.
   * @param time the time value.
   * @return the pose of the base motion.
   */
  Pose evaluatePose(const double time) const;

  /*!
   * Get target pose.
   * @return target pose.
   */
  const Pose getTarget() const;

  /*!
   * Evaluate the base twist at a given time.
   * @param time the time to evaluate the twist at.
   * @return the twist of the base in the defined frame id.
   */
  Twist evaluateTwist(const double time) const;

  friend std::ostream& operator << (std::ostream& out, const BaseTarget& baseTarget);
  friend class StepCompleter;
  friend class StepRosConverter;

 protected:
  bool ignoreTimingOfLegMotion_;
  double averageLinearVelocity_;
  double averageAngularVelocity_;
  double minimumDuration_;

 private:

  /*!
   * Computes the internal trajectory based on the profile type.
   */
  void computeDuration(const Step& step, const AdapterBase& adapter);
  bool computeTrajectory();

  Pose start_;
  Pose target_;
  std::string frameId_;
  double duration_;
  ControlSetup controlSetup_;

  //! Base trajectory.
  curves::CubicHermiteSE3Curve trajectory_;

  //! If trajectory is updated.
  bool isComputed_;
};

} /* namespace */
