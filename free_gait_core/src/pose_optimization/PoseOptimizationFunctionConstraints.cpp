/*
 * PoseOptimizationFunctionConstraints.cpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#include "free_gait_core/pose_optimization/PoseOptimizationFunctionConstraints.hpp"
#include "free_gait_core/pose_optimization/PoseParameterization.hpp"

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

void PoseOptimizationFunctionConstraints::setSupportRegion(const grid_map::Polygon& supportRegion)
{
  supportRegion_ = supportRegion;
  nInequalityConstraints_ = supportRegion.nVertices();
  supportRegion_.convertToInequalityConstraints(supportRegionInequalityConstraintGlobalJacobian_,
                                                supportRegionInequalityConstraintsMaxValues_);
}

const grid_map::Polygon& PoseOptimizationFunctionConstraints::getSupportRegion() const
{
  return supportRegion_;
}

bool PoseOptimizationFunctionConstraints::getGlobalBoundConstraintMinValues(
    numopt_common::Vector& values)
{
  values = Eigen::VectorXd::Constant(PoseParameterization::getGlobalSizeStatic(), std::numeric_limits<double>::lowest());
  return true;
}

bool PoseOptimizationFunctionConstraints::getGlobalBoundConstraintMaxValues(
    numopt_common::Vector& values)
{
  values = Eigen::VectorXd::Constant(PoseParameterization::getGlobalSizeStatic(), std::numeric_limits<double>::max());
  return true;
}

bool PoseOptimizationFunctionConstraints::getInequalityConstraintValues(numopt_common::Vector& values,
                                                                        const numopt_common::Parameterization& p,
                                                                        bool newParams)
{
  const auto& poseParameterization = dynamic_cast<const PoseParameterization&>(p);
  const Position basePosition = poseParameterization.getPosition();
  values = supportRegionInequalityConstraintGlobalJacobian_ * basePosition.vector().head(2);
  return true;
}

bool PoseOptimizationFunctionConstraints::getInequalityConstraintMinValues(numopt_common::Vector& d)
{
  d = numopt_common::Vector::Constant(nInequalityConstraints_, std::numeric_limits<double>::lowest());
  return true;
}

bool PoseOptimizationFunctionConstraints::getInequalityConstraintMaxValues(numopt_common::Vector& f)
{
  f = supportRegionInequalityConstraintsMaxValues_;
  return true;
}

} /* namespace */
