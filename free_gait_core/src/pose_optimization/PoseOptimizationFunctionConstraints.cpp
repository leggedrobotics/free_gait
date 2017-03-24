/*
 * PoseOptimizationFunctionConstraints.cpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#include "free_gait_core/pose_optimization/PoseOptimizationFunctionConstraints.hpp"

namespace free_gait {

PoseOptimizationFunctionConstraints::PoseOptimizationFunctionConstraints()
    : NonlinearFunctionConstraints()
{
  nEqualityConstraints_ = 0;
  nInequalityConstraints_ = 0;
}

PoseOptimizationFunctionConstraints::~PoseOptimizationFunctionConstraints()
{
}

bool PoseOptimizationFunctionConstraints::getGlobalBoundConstraintMinValues(
    numopt_common::Vector& values)
{
  values = -std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(4);
  return true;
}

bool PoseOptimizationFunctionConstraints::getGlobalBoundConstraintMaxValues(
    numopt_common::Vector& values)
{
  values = std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(4);
  return true;
}

} /* namespace */
