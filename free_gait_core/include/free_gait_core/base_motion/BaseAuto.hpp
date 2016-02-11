/*
 * BaseAuto.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/executor/AdapterBase.hpp"
#include "free_gait_core/base_motion/BaseMotionBase.hpp"
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/step/Step.hpp"
#include "free_gait_core/step/StepQueue.hpp"
#include "free_gait_core/pose_optimization/PoseOptimization.hpp"

// STD
#include <string>
#include <memory>

// Curves
#include "curves/CubicHermiteSE3Curve.hpp"

namespace free_gait {

class StepRosConverter;
class StepCompleter;

class BaseAuto : public BaseMotionBase
{
 public:
  typedef typename curves::CubicHermiteSE3Curve::ValueType ValueType;
  typedef typename curves::Time Time;

  BaseAuto();
  virtual ~BaseAuto();

  BaseAuto(const BaseAuto& other);

  std::unique_ptr<BaseMotionBase> clone() const;

  const ControlSetup getControlSetup() const;

  /*!
   * Update the profile with the base start pose.
   * Do this to avoid jumps of the base.
   * @param startPose the start pose of the base in the frameId_ frame.
   */
  void updateStartPose(const Pose& startPose);

  bool compute(const State& state, const Step& step, const StepQueue& queue, const AdapterBase& adapter);

  /*!
   * Returns the total duration of the trajectory.
   * @return the duration.
   */
  double getDuration() const;

  /*!
   * Evaluate the base shift pose at a given time.
   * @param time the time value.
   * @return the pose of the base shift trajectory.
   */
  Pose evaluatePose(const double time) const;
  Twist evaluateTwist(const double time) const;

  friend std::ostream& operator << (std::ostream& out, const BaseAuto& baseAuto);
  friend class StepCompleter;
  friend class StepRosConverter;

 protected:
  std::unique_ptr<double> height_; // In control frame.
  double averageLinearVelocity_;
  double averageAngularVelocity_;
  double supportMargin_;

 private:

  bool computeHeight(const State& state, const AdapterBase& adapter);
  bool generateFootholdLists(const State& state, const Step& step, const StepQueue& queue, const AdapterBase& adapter);
  void getAdaptiveHorizontalTargetPosition(const State& state, const AdapterBase& adapter, Position& horizontalTargetPositionInWorldFrame);
  void getAdaptiveTargetPose(const State& state, const AdapterBase& adapter, const Position& horizontalTargetPositionInWorld, Pose& targetPoseInWorld);
  void computeDuration();
  bool optimizePose(Pose& pose);

  /*!
   * Computes the internal trajectory based on the profile type.
   */
  bool computeTrajectory();

  Pose start_; // In world frame.
  Pose target_; // In world frame.
  double duration_;
  PlanarStance nominalPlanarStanceInBaseFrame_;

  ControlSetup controlSetup_;

  //! Base trajectory.
  curves::CubicHermiteSE3Curve trajectory_;

  Stance footholdsToReach_, footholdsInSupport_, nominalStanceInBaseFrame_;
  Stance footholdsForTerrain_; // TODO Replace with full optimization.
  PoseOptimization poseOptimization_;

  //! If trajectory is updated.
  bool computed_;
};

} /* namespace */
