/*
 * PoseOptimizationQP.hpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationBase.hpp"

#include <grid_map_core/Polygon.hpp>
#include <numopt_common/QuadraticProblemSolver.hpp>

#include <unordered_map>
#include <memory>


namespace free_gait {

class PoseOptimizationQP : public PoseOptimizationBase
{
 public:
  PoseOptimizationQP(const AdapterBase& adapter);
  virtual ~PoseOptimizationQP();
  PoseOptimizationQP(const PoseOptimizationQP& other);

  /*!
   * Computes the optimized pose.
   * @param[in/out] pose the pose to optimize from the given initial guess.
   * @return true if successful, false otherwise.
   */
  bool optimize(Pose& pose);

 private:
  std::unique_ptr<numopt_common::QuadraticProblemSolver> solver_;
  unsigned int nStates_;
  unsigned int nDimensions_;
};

} /* namespace loco */
