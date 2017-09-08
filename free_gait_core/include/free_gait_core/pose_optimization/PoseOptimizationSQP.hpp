/*
 * PoseOptimizationSQP.hpp
 *
 *  Created on: Mar 21, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/executor/AdapterBase.hpp"
#include "free_gait_core/executor/State.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationBase.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationObjectiveFunction.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationFunctionConstraints.hpp"

#include <grid_map_core/Polygon.hpp>
#include <numopt_common/QuadraticProblemSolver.hpp>
#include <robot_utils/timers/ChronoTimer.hpp>

#include <limits>

namespace free_gait {

class PoseOptimizationSQP : public PoseOptimizationBase
{
 public:
  using OptimizationStepCallbackFunction = std::function<void(const size_t, const State&, const double, const bool)>;
  using LimbLengths = PoseOptimizationFunctionConstraints::LimbLengths;

  //! Constructor. Keeps are reference to the adapter, be careful when multi-threading!
  //! @param adapter the adapter to the robot data.
  PoseOptimizationSQP(const AdapterBase& adapter);
  virtual ~PoseOptimizationSQP();

  void setCurrentState(const State& state);

  /*!
   * Registers a callback function that is called after every iteration step
   * of the minimizer. Use this method for debugging and visualizations or leave
   * it unregistered if not used.
   * @param callback the callback method to be registered.
   */
  void registerOptimizationStepCallback(OptimizationStepCallbackFunction callback);

  /*!
   * Computes the optimized pose with SQP.
   * @param[in/out] pose the optimized pose from the provided initial pose.
   * @return true if successful, false otherwise.
   */
  bool optimize(Pose& pose);

  void optimizationStepCallback(const size_t iterationStep, const numopt_common::Parameterization& parameters,
                                const double functionValue, const bool finalIteration);

  void callExternalOptimizationStepCallbackWithPose(const Pose& pose, const size_t iterationStep,
                                                    const double functionValue = std::numeric_limits<double>::max(),
                                                    const bool finalIteration = false);

  /*!
   * Return the duration of the last optimization in micro seconds.
   * @return the duration in micro seconds.
   */
  double getOptimizationDuration() const;

 private:
  void callExternalOptimizationStepCallback(const size_t iterationStep, const double functionValue =
                                                std::numeric_limits<double>::max(),
                                            const bool finalIteration = false);

  State originalState_;

  //! State being optimized.
  State state_;

  std::shared_ptr<PoseOptimizationObjectiveFunction> objective_;
  std::shared_ptr<PoseOptimizationFunctionConstraints> constraints_;
  OptimizationStepCallbackFunction optimizationStepCallback_;
  robot_utils::HighResolutionClockTimer timer_;
  double durationInCallback_;
};

} /* namespace loco */
