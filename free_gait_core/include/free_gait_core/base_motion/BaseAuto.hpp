/*
 * BaseAuto.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/executor/AdapterBase.hpp"
#include "free_gait_core/base_motion/BaseMotionBase.hpp"
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/step/Step.hpp"
#include "free_gait_core/step/StepQueue.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationGeometric.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationQP.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationSQP.hpp"
#include "free_gait_core/pose_optimization/PoseConstraintsChecker.hpp"

#include <curves/CubicHermiteSE3Curve.hpp>

#include <string>
#include <memory>

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

  bool prepareComputation(const State& state, const Step& step, const StepQueue& queue, const AdapterBase& adapter);
  bool needsComputation() const;
  bool isComputed() const;
  void reset();

  /*!
   * Returns the total duration of the trajectory.
   * @return the duration.
   */
  double getDuration() const;

  const std::string& getFrameId(const ControlLevel& controlLevel) const;

  void setHeight(const double height);
  double getHeight() const;
  void setAverageLinearVelocity(const double averageLinearVelocity);
  double getAverageLinearVelocity() const;
  void setAverageAngularVelocity(const double averageAngularVelocity);
  double getAverageAngularVelocity() const;
  double getSupportMargin() const;
  void setSupportMargin(double supportMargin);
  void setTolerateFailingOptimization(const bool tolerateFailingOptimization);

  /*!
   * Evaluate the base pose at a given time.
   * @param time the time evakyate the pose at.
   * @return the pose of the base in the defined frame id.
   */
  Pose evaluatePose(const double time) const;

  /*!
   * Evaluate the base twist at a given time.
   * @param time the time to evaluate the twist at.
   * @return the twist of the base in the defined frame id.
   */
  Twist evaluateTwist(const double time) const;

  friend std::ostream& operator << (std::ostream& out, const BaseAuto& baseAuto);

  friend class StepCompleter;
  friend class StepRosConverter;

 protected:
  std::string frameId_;
  std::unique_ptr<double> height_; // In control frame.
  PoseOptimizationBase::LimbLengths minLimbLenghts_, maxLimbLenghts_;
  bool ignoreTimingOfLegMotion_;
  double averageLinearVelocity_;
  double averageAngularVelocity_;
  double supportMargin_;
  double minimumDuration_;

 private:

  bool computeHeight(const State& state, const StepQueue& queue, const AdapterBase& adapter);
  bool generateFootholdLists(const State& state, const Step& step, const StepQueue& queue, const AdapterBase& adapter);
  void computeDuration(const Step& step, const AdapterBase& adapter);
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

  // In world frame.
  Stance footholdsToReach_, footholdsInSupport_, footholdsForOrientation_, footholdsOfNextLegMotion_;
  // In base frame.
  Stance nominalStanceInBaseFrame_;

  //! If trajectory is updated.
  bool isComputed_;

  bool tolerateFailingOptimization_;

  //! Optimizers.
  std::unique_ptr<PoseOptimizationGeometric> poseOptimizationGeometric_;
  std::unique_ptr<PoseOptimizationQP> poseOptimizationQP_;
  std::unique_ptr<PoseOptimizationSQP> poseOptimizationSQP_;
  std::unique_ptr<PoseConstraintsChecker> constraintsChecker_;

};

} /* namespace */
