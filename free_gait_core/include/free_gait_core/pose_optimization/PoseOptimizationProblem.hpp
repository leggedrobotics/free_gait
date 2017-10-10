/*
 * PoseOptimizationProblem.hpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include "free_gait_core/pose_optimization/PoseOptimizationObjectiveFunction.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationFunctionConstraints.hpp"

#include <numopt_common/ConstrainedNonlinearProblem.hpp>

namespace free_gait {

class PoseOptimizationProblem : public numopt_common::ConstrainedNonlinearProblem
{
 public:
  PoseOptimizationProblem(std::shared_ptr<PoseOptimizationObjectiveFunction> objectiveFunction,
                          std::shared_ptr<PoseOptimizationFunctionConstraints> functionConstraints);
  virtual ~PoseOptimizationProblem();
};

} /* namespace */
