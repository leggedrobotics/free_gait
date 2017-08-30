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

PoseOptimizationSQP::PoseOptimizationSQP(const AdapterBase& adapter)
    : adapter_(adapter),
      timer_("PoseOptimizationSQP"),
      durationInCallback_(0.0)
{
  objective_.reset(new PoseOptimizationObjectiveFunction());

  constraints_.reset(new PoseOptimizationFunctionConstraints());
  PoseOptimizationFunctionConstraints::LegPositions positionsBaseToHipInBaseFrame;
  for (const auto& limb : adapter_.getLimbs()) {
    positionsBaseToHipInBaseFrame[limb] = adapter_.getPositionBaseToHipInBaseFrame(limb);
  }
  constraints_->setPositionBaseToHip(positionsBaseToHipInBaseFrame);

  timer_.setAlpha(1.0);
}

PoseOptimizationSQP::~PoseOptimizationSQP()
{
}

void PoseOptimizationSQP::setCurrentState(const State& state)
{
  originalState_ = state;
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
  objective_->setSupportRegion(supportRegion);
}

void PoseOptimizationSQP::setLimbLengthConstraints(const LegLengths& minLimbLenghts, const LegLengths& maxLimbLenghts)
{
  constraints_->setLimbLengthConstraints(minLimbLenghts, maxLimbLenghts);
}

void PoseOptimizationSQP::registerOptimizationStepCallback(OptimizationStepCallbackFunction callback)
{
  optimizationStepCallback_ = callback;
}

bool PoseOptimizationSQP::optimize(Pose& pose)
{
  timer_.pinTime("total");
  durationInCallback_ = 0.0;
  state_ = originalState_;
  checkSupportRegion();

  // Compute initial solution.
  computeInitialSolution(pose);
  objective_->setInitialPose(pose);
  state_.setPoseBaseToWorld(pose);
  adapter_.setInternalDataFromState(state_); // To guide IK.
  updateJointPositionsInState(state_); // For CoM calculation.
  adapter_.setInternalDataFromState(state_);
  const Position centerOfMassInBaseFrame(adapter_.transformPosition(adapter_.getWorldFrameId(), adapter_.getBaseFrameId(),
                                 adapter_.getCenterOfMassInWorldFrame()));
  objective_->setCenterOfMass(centerOfMassInBaseFrame);
  constraints_->setCenterOfMass(centerOfMassInBaseFrame);
  callExternalOptimizationStepCallback(0);

  // Optimize.
  PoseOptimizationProblem problem(objective_, constraints_);
  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver(
      new numopt_ooqp::QPFunctionMinimizer);
//  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver(
//      new numopt_quadprog::ActiveSetFunctionMinimizer);
//  numopt_sqp::SQPFunctionMinimizer solver(qpSolver, 1000, 0.001, 3, -DBL_MAX);
  numopt_sqp::SQPFunctionMinimizer solver(qpSolver, 1000, 0.01, 3, -DBL_MAX);
  solver.registerOptimizationStepCallback(
      std::bind(&PoseOptimizationSQP::optimizationStepCallback, this, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4));
  solver.setCheckConstraints(false);
  solver.setPrintOutput(true);
  PoseParameterization params;
  params.setPose(pose);
  double functionValue;
  if (!solver.minimize(&problem, params, functionValue)) return false;
  pose = params.getPose();
  // TODO Fix unit quaternion?

  adapter_.setInternalDataFromState(originalState_);
  timer_.splitTime("total");
  return true;
}

void PoseOptimizationSQP::optimizationStepCallback(const size_t iterationStep,
                                                   const numopt_common::Parameterization& parameters,
                                                   const double functionValue,
                                                   const bool finalIteration)
{
  auto& poseParameterization = dynamic_cast<const PoseParameterization&>(parameters);

  // Update center of mass. // TODO Make optional.
  state_.setPoseBaseToWorld(poseParameterization.getPose());
  state_.setAllJointPositions(originalState_.getJointPositions());
  adapter_.setInternalDataFromState(state_);
  updateJointPositionsInState(state_);
  adapter_.setInternalDataFromState(state_); // TODO Improve efficiency.
  const Position centerOfMassInBaseFrame(adapter_.transformPosition(adapter_.getWorldFrameId(), adapter_.getBaseFrameId(),
                                 adapter_.getCenterOfMassInWorldFrame()));
  objective_->setCenterOfMass(centerOfMassInBaseFrame);
  constraints_->setCenterOfMass(centerOfMassInBaseFrame);

  callExternalOptimizationStepCallback(iterationStep + 1, functionValue, finalIteration);
}

double PoseOptimizationSQP::getOptimizationDuration() const
{
  return timer_.getAverageElapsedTimeUSec("total") - durationInCallback_;
}

void PoseOptimizationSQP::checkSupportRegion()
{
  // If no support polygon region, use positions.
  if (constraints_->getSupportRegion().nVertices() == 0) {
    grid_map::Polygon supportRegion;
    for (const auto& foot : stance_)
      supportRegion.addVertex(foot.second.vector().head<2>());
  }
}

const void PoseOptimizationSQP::computeInitialSolution(Pose& pose)
{
  checkSupportRegion();
  pose.setIdentity();

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
  A = A / ((double) stance_.size()); // Error in (46).
  C -= stance_.size() * A * A; // See (45).
  Eigen::EigenSolver<Eigen::Matrix4d> eigenSolver(C);
  int maxCoeff;
  eigenSolver.eigenvalues().real().maxCoeff(&maxCoeff);
  // Eigen vector corresponding to max. eigen value.
  pose.getRotation() = RotationQuaternion(eigenSolver.eigenvectors().col(maxCoeff).real());
  pose.getRotation().setUnique();

  // Apply roll/pitch adaptation factor (~0.5).
  const RotationQuaternion yawRotation(RotationVector(RotationVector(pose.getRotation()).vector().cwiseProduct(Eigen::Vector3d::UnitZ())));
  const RotationQuaternion rollPitchRotation(RotationVector(0.5 * RotationVector(pose.getRotation() * yawRotation.inverted()).vector()));
  pose.getRotation() = yawRotation * rollPitchRotation;
}

void PoseOptimizationSQP::updateJointPositionsInState(State& state) const
{
  for (const auto& foot : stance_) {
    const Position footPositionInBase(
        adapter_.transformPosition(adapter_.getWorldFrameId(), adapter_.getBaseFrameId(), foot.second));
    JointPositionsLeg jointPositions;
    const bool success = adapter_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(footPositionInBase, foot.first, jointPositions);
    if (success) state.setJointPositionsForLimb(foot.first, jointPositions);
  }
}

void PoseOptimizationSQP::callExternalOptimizationStepCallback(const size_t iterationStep, const double functionValue,
                                                               const bool finalIteration)
{
  if (optimizationStepCallback_) {
    timer_.pinTime("callback");
    adapter_.setInternalDataFromState(state_);
    State previewState(state_);
    updateJointPositionsInState(previewState);
    optimizationStepCallback_(iterationStep, previewState, functionValue, finalIteration);
    timer_.splitTime("callback");
    durationInCallback_ += timer_.getAverageElapsedTimeUSec("callback");
  }
}

} /* namespace */
