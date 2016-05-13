/*
 * BaseTrajectory.hpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/base_motion/BaseMotionBase.hpp"
#include "free_gait_core/TypeDefs.hpp"
#include "curves/CubicHermiteSE3Curve.hpp"

// STD
#include <memory>
#include <string>

namespace free_gait {

class StepRosConverter;
class StepCompleter;

/*!
 * Implementation of a base trajectory as polynomial spline.
 */
class BaseTrajectory : public BaseMotionBase
{
 public:
  typedef typename curves::CubicHermiteSE3Curve TrajectoryType;
  typedef typename curves::CubicHermiteSE3Curve::ValueType ValueType;
  typedef typename curves::CubicHermiteSE3Curve::DerivativeType DerivativeType;
  typedef typename curves::Time Time;

  /*!
   * Constructor.
   */
  BaseTrajectory();

  /*!
   * Destructor.
   */
  virtual ~BaseTrajectory();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  virtual std::unique_ptr<BaseMotionBase> clone() const;

  const ControlSetup getControlSetup() const;

  /*!
   * Update the trajectory with the base start pose.
   * Do this to avoid jumps of the base.
   * @param startPose the start pose of the base in the frameId_ frame.
   */
  void updateStartPose(const Pose& startPose);

  bool prepareComputation(const State& state, const Step& step, const StepQueue& queue,
                          const AdapterBase& adapter);
  bool needsComputation() const;
  bool isComputed() const;

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
   * Evaluate the base motion at a given time.
   * @param time the time value.
   * @return the pose of the base.
   */
  Pose evaluatePose(const double time) const;

  /*!
   * Print the contents to console for debugging.
   * @param out the output stream.
   * @param baseTrajectory the base shift trajectory to debug.
   * @return the resulting output stream.
   */
  friend std::ostream& operator << (std::ostream& out, const BaseTrajectory& baseTrajectory);

  friend class StepCompleter;
  friend class StepRosConverter;

 protected:

  bool isComputed_;
  double duration_;
  ControlSetup controlSetup_;

  //! Knots.
  std::unordered_map<ControlLevel, std::string, EnumClassHash> frameIds_;
  std::unordered_map<ControlLevel, std::vector<Time>, EnumClassHash> times_;
  std::unordered_map<ControlLevel, std::vector<ValueType>, EnumClassHash> values_;
  std::unordered_map<ControlLevel, std::vector<DerivativeType>, EnumClassHash> derivatives_;

  //! Base trajectory.
  TrajectoryType trajectory_;
  // TODO Add force/torque trajectory.
};

} /* namespace */
