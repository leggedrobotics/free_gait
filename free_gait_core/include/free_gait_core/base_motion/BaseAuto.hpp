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
#include "free_gait_core/pose_optimization/PoseOptimization.hpp"

#include <string>

// Curves
#include "curves/CubicHermiteSE3Curve.hpp"

namespace free_gait {

class StepRosConverter;

class BaseAuto : public BaseMotionBase
{
 public:
  typedef typename curves::CubicHermiteSE3Curve::ValueType ValueType;
  typedef typename curves::Time Time;

  BaseAuto(const State& state, const Step& step, const AdapterBase& adapter);
  virtual ~BaseAuto();

  const ControlSetup getControlSetup() const;

  /*!
   * Update the profile with the base start pose.
   * Do this to avoid jumps of the base.
   * @param startPose the start pose of the base in the frameId_ frame.
   * @return true if successful, false otherwise.
   */
  void updateStartPose(const Pose& startPose);

  bool compute();

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

  friend std::ostream& operator << (std::ostream& out, const BaseAuto& baseAuto);
  friend class StepRosConverter;

 protected:
  double height_; // In control frame.
  double averageVelocity_;
  double supportSafetyMargin_;

 private:

  bool generateFootholdLists();
  void getAdaptiveHorizontalTargetPosition(Position& horizontalTargetPositionInWorldFrame);
  void getAdaptiveTargetPose(const Position& horizontalTargetPositionInWorld, Pose& targetPoseInWorld);
  void optimizePose(Pose& pose);

  /*!
   * Computes the internal trajectory based on the profile type.
   */
  bool computeTrajectory();

  Pose start_; // In world frame.
  Pose target_; // In world frame.
  double duration_;

  ControlSetup controlSetup_;

  //! Base trajectory.
  curves::CubicHermiteSE3Curve trajectory_;

  PoseOptimization::FeetPositions footholdsToReach_, footholdsInSupport_, desiredFeetPositionsInBase_;
  PoseOptimization poseOptimization_;

  //! If trajectory is updated.
  bool trajectoryUpdated_;

  const State& state_;
  const Step& step_;
  const AdapterBase& adapter_;
};

} /* namespace */
