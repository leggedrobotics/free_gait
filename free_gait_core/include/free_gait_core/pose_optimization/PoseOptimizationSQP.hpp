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

// Grid map
#include <grid_map_core/Polygon.hpp>

// Numerical Optimization
#include <numopt_common/QuadraticProblemSolver.hpp>

namespace free_gait {

class PoseOptimizationSQP
{
 public:
  PoseOptimizationSQP(const AdapterBase& adapter, const State& state);
  virtual ~PoseOptimizationSQP();

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
   * Computes the optimized pose.
   * @param[in/out] pose the pose to optimize from the given initial guess.
   * @return true if successful, false otherwise.
   */
  bool optimize(Pose& pose);

 private:
  const AdapterBase& adapter_;
  const State& state_;
  std::shared_ptr<PoseOptimizationObjectiveFunction> objective_;
  std::shared_ptr<PoseOptimizationFunctionConstraints> constraints_;
  unsigned int nStates_;
  unsigned int nDimensions_;
};

} /* namespace loco */
