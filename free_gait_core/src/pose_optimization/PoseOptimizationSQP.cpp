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
#include <message_logger/message_logger.hpp>
#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>
#include <numopt_ooqp/QPFunctionMinimizer.hpp>
#include <numopt_sqp/SQPFunctionMinimizer.hpp>

#include <functional>

namespace free_gait {

PoseOptimizationSQP::PoseOptimizationSQP(const AdapterBase& adapter, const State& state)
    : adapter_(adapter),
      state_(state),
      nStates_(4),
      nDimensions_(3)
{
  objective_.reset(new PoseOptimizationObjectiveFunction(adapter_, state_));
  constraints_.reset(new PoseOptimizationFunctionConstraints(adapter_, state_));
}

PoseOptimizationSQP::~PoseOptimizationSQP()
{
}

void PoseOptimizationSQP::setStance(const Stance& stance)
{
  stance_ = stance;
  objective_->setStance(stance);
  constraints_->setStance(stance);
}

void PoseOptimizationSQP::setNominalStance(
    const Stance& nominalStanceInBaseFrame)
{
  objective_->setNominalStance(nominalStanceInBaseFrame);
}

void PoseOptimizationSQP::setSupportRegion(const grid_map::Polygon& supportRegion)
{
  constraints_->setSupportRegion(supportRegion);
}

void PoseOptimizationSQP::registerOptimizationStepCallback(OptimizationStepCallbackFunction callback)
{
  optimizationStepCallback_ = callback;
}

bool PoseOptimizationSQP::optimize(Pose& pose)
{
  // If no support polygon provided, use positions.
  if (constraints_->getSupportRegion().nVertices() == 0) {
    grid_map::Polygon supportRegion;
    for (const auto& foot : objective_->getStance())
      supportRegion.addVertex(foot.second.vector().head<2>());
  }

  // Optimize.
  PoseOptimizationProblem problem(objective_, constraints_);
  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver(
      new numopt_ooqp::QPFunctionMinimizer);
//  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver(
//      new numopt_quadprog::ActiveSetFunctionMinimizer);
  numopt_sqp::SQPFunctionMinimizer solver(qpSolver, 100, 0.0001, 3, -DBL_MAX, true, true);
  if (optimizationStepCallback_) {
    solver.registerOptimizationStepCallback(
        std::bind(&PoseOptimizationSQP::optimizationStepCallback, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));
  }
  solver.setCheckConstraints(false);
  solver.setPrintOutput(true);
  PoseParameterization params;
  params.setPose(pose);
  double functionValue;
  if (!solver.minimize(&problem, params, functionValue)) return false;
  pose = params.getPose();
  // TODO Fix unit quaternion?
  return true;
}

void PoseOptimizationSQP::optimizationStepCallback(
    const size_t iterationStep, const numopt_common::Parameterization& parameters,
    const double functionValue)
{
  if (!optimizationStepCallback_) return;
  auto& poseParameterization = dynamic_cast<const PoseParameterization&>(parameters);
  State previewState(state_);
  previewState.setPoseBaseToWorld(poseParameterization.getPose());
  for (const auto& limb : adapter_.getLimbs()) {
    const Position footPositionInWorld(
        adapter_.transformPosition(adapter_.getWorldFrameId(), adapter_.getBaseFrameId(),
                                   stance_.at(limb)));
    JointPositionsLeg jointPositions;
    adapter_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(footPositionInWorld, limb, jointPositions);
    previewState.setJointPositionsForLimb(limb, jointPositions);
  }
  optimizationStepCallback_(iterationStep, previewState, functionValue);
//  MELO_INFO_STREAM("parameters" << poseParameterization.getPose());
}

} /* namespace */
