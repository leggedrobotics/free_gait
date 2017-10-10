/*
 * PoseOptimizationProblem.cpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */
#include "free_gait_core/pose_optimization/PoseOptimizationProblem.hpp"

namespace free_gait {

PoseOptimizationProblem::PoseOptimizationProblem(
    std::shared_ptr<PoseOptimizationObjectiveFunction> objectiveFunction,
    std::shared_ptr<PoseOptimizationFunctionConstraints> functionConstraints)
    : ConstrainedNonlinearProblem(objectiveFunction, functionConstraints)
{
}

PoseOptimizationProblem::~PoseOptimizationProblem()
{
}

} /* namespace */
