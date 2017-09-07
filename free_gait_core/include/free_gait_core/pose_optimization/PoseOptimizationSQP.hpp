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
#include "free_gait_core/pose_optimization/PoseOptimizationObjectiveFunction.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationFunctionConstraints.hpp"

#include <grid_map_core/Polygon.hpp>
#include <numopt_common/QuadraticProblemSolver.hpp>
#include <robot_utils/timers/ChronoTimer.hpp>

#include <limits>

namespace free_gait {

class PoseOptimizationSQP
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
   * Set the positions of the feet of the robot in world coordinate system.
   * @param feetPositions the feet positions.
   */
  void setStance(const Stance& stance);

  /*!
   * Define the desired leg configuration by specifying the desired feet positions
   * relative to the base.
   * @param desiredFeetPositionsInBase the desired feet positions in base frame.
   */
  void setNominalStance(const Stance& nominalStanceInBaseFrame);

  /*!
   * Set the support polygon for constraining the pose optimization.
   * If support polygon is not set, the convex hull of all feet positions is used.
   * @param supportPolygon the support polygon as a list of vertices.
   */
  void setSupportRegion(const grid_map::Polygon& supportRegion);

  /*!
   * Set the max. and min. value for constraining the limb length.
   * Note: minLimbLenghts and maxLimbLenghts need to have same length!
   * @param minLimbLenghts the min. limb length.
   * @param maxLimbLenghts the max. limb length.
   */
  void setLimbLengthConstraints(const LimbLengths& minLimbLenghts, const LimbLengths& maxLimbLenghts);

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
  void checkSupportRegion();

  void updateJointPositionsInState(State& state) const;

  void callExternalOptimizationStepCallback(const size_t iterationStep, const double functionValue =
                                                std::numeric_limits<double>::max(),
                                            const bool finalIteration = false);

  const AdapterBase& adapter_;
  State originalState_;

  //! State being optimized.
  State state_;

  Stance stance_;
  std::shared_ptr<PoseOptimizationObjectiveFunction> objective_;
  std::shared_ptr<PoseOptimizationFunctionConstraints> constraints_;
  OptimizationStepCallbackFunction optimizationStepCallback_;
  robot_utils::HighResolutionClockTimer timer_;
  double durationInCallback_;
};

} /* namespace loco */
