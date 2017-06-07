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
#include <Eigen/Eigenvalues>
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

const Pose PoseOptimizationSQP::computeInitialSolution() const
{
  // If no support polygon provided, use positions.
  if (constraints_->getSupportRegion().nVertices() == 0) {
    grid_map::Polygon supportRegion;
    for (const auto& foot : stance_)
      supportRegion.addVertex(foot.second.vector().head<2>());
  }

  Pose pose; // Identity.

  // Planar position: Use geometric center of support region.
  Position center;
  center.vector().head(2) = constraints_->getSupportRegion().getCentroid();

  // Height: Average of stance plus default height.
  for (const auto& foot : stance_) {
    center.z() += foot.second.z() - objective_->getNominalStance().at(foot.first).z();
  }

  center.z() /= (double) stance_.size();
  pose.getPosition() = center;

  // Orientation: Squared error minimization (see sec. 4.2.2 from Bloesch, Technical
  // Implementations of the Sense of Balance, 2016).
  // Notes on (38):
  // - i: Identity pose (I),
  // - j: Solution (B),
  // - t = I_r_IB,
  // - q = q_IB,
  // - a_k = I_f_k: Foot position for leg k in inertial frame,
  // - b_k = B_\hat_f_K: Nominal foot position for leg k in base frame.

  Eigen::Matrix4d C(Eigen::Matrix4d::Zero()); // See (45).
  Eigen::Matrix4d A(Eigen::Matrix4d::Zero()); // See (46).
  for (const auto& foot : stance_) {
    const RotationQuaternion footPositionInertialFrame(0.0, foot.second.vector()); // \bar_a_k = S^T * a_k (39).
    const RotationQuaternion defaultFootPositionBaseFrame(0.0, objective_->getNominalStance().at(foot.first).vector()); // \bar_b_k = S^T * b_k.
    Eigen::Matrix4d Ak = footPositionInertialFrame.getQuaternionMatrix() - defaultFootPositionBaseFrame.getConjugateQuaternionMatrix(); // See (46).
    C += Ak * Ak; // See (45).
    A += Ak; // See (46).
  }
  A = A / ((double) stance_.size());
  C -= stance_.size() * A * A; // See (45).
  Eigen::EigenSolver<Eigen::Matrix4d> eigenSolver(C);
  int maxCoeff;
  eigenSolver.eigenvalues().real().maxCoeff(&maxCoeff);
  // Eigen vector corresponding to max. eigen value.
  pose.getRotation() = RotationQuaternion(eigenSolver.eigenvectors().col(maxCoeff).real());
  pose.getRotation().setUnique();

  return pose;
}

bool PoseOptimizationSQP::optimize(Pose& pose)
{
  // Optimize.
  PoseOptimizationProblem problem(objective_, constraints_);
  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver(
      new numopt_ooqp::QPFunctionMinimizer);
//  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver(
//      new numopt_quadprog::ActiveSetFunctionMinimizer);
  numopt_sqp::SQPFunctionMinimizer solver(qpSolver, 100, 0.0001, 5, -DBL_MAX, true, true);
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
  adapter_.setInternalDataFromState(previewState);
//  std::cout << "pose: " << previewState.getPositionWorldToBaseInWorldFrame() << std::endl;
  for (const auto& limb : adapter_.getLimbs()) {
    const Position footPositionInBase(
        adapter_.transformPosition(adapter_.getWorldFrameId(), adapter_.getBaseFrameId(),
                                   stance_.at(limb)));
//    std::cout << "footPositionInBase:" << footPositionInBase << std::endl;
    JointPositionsLeg jointPositions;
    bool success = adapter_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(footPositionInBase, limb, jointPositions);
//    std::cout << "success:" << success << std::endl;
//    std::cout << "jointPositions:" << jointPositions << std::endl;
    previewState.setJointPositionsForLimb(limb, jointPositions);
  }
  adapter_.setInternalDataFromState(state_);
  optimizationStepCallback_(iterationStep, previewState, functionValue);
//  MELO_INFO_STREAM("parameters" << poseParameterization.getPose());
}

} /* namespace */
