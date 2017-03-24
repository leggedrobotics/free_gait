/*
 * PoseOptimizationSQP.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#include "free_gait_core/pose_optimization/PoseOptimizationSQP.hpp"
#include "free_gait_core/pose_optimization/PoseParameterization.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationProblem.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <kindr/Core>
#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>
#include <numopt_ooqp/QPFunctionMinimizer.hpp>
#include <numopt_sqp/SQPFunctionMinimizer.hpp>

namespace free_gait {

PoseOptimizationSQP::PoseOptimizationSQP(const AdapterBase& adapter, const State& state)
    : adapter_(adapter),
      state_(state),
      nStates_(4),
      nDimensions_(3)
{
  objective_.reset(new PoseOptimizationObjectiveFunction(adapter_, state_));
  constraints_.reset(new PoseOptimizationFunctionConstraints());
}

PoseOptimizationSQP::~PoseOptimizationSQP()
{
}

void PoseOptimizationSQP::setStance(const Stance& stance)
{
  objective_->setStance(stance);
}

void PoseOptimizationSQP::setNominalStance(
    const Stance& nominalStanceInBaseFrame)
{
  objective_->setNominalStance(nominalStanceInBaseFrame);
}

void PoseOptimizationSQP::setSupportRegion(const grid_map::Polygon& supportRegion)
{
//  supportRegion_ = supportRegion;
}

bool PoseOptimizationSQP::optimize(Pose& pose)
{
  PoseOptimizationProblem problem(objective_, constraints_);
  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver(
      new numopt_ooqp::QPFunctionMinimizer);
//  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver(
//      new numopt_quadprog::ActiveSetFunctionMinimizer);
  numopt_sqp::SQPFunctionMinimizer solver(qpSolver, 10, 0.005, 5, -DBL_MAX, true, true);
  solver.setCheckConstraints(false);
  solver.setPrintOutput(false);
  PoseParameterization params;
  params.setPose(pose);
  double functionValue;
  if (!solver.minimize(&problem, params, functionValue)) return false;
  pose = params.getPose();
  // TODO Fix unit quaternion?
  return true;
}

} /* namespace */
