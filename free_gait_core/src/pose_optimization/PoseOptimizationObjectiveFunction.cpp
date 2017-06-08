/*
 * PoseOptimizationObjectiveFunction.cpp
 *
 *  Created on: Mar 22, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#include "free_gait_core/pose_optimization/PoseOptimizationObjectiveFunction.hpp"
#include "free_gait_core/pose_optimization/PoseParameterization.hpp"

namespace free_gait {

PoseOptimizationObjectiveFunction::PoseOptimizationObjectiveFunction(const AdapterBase& adapter, const State& state)
    : NonlinearObjectiveFunction(),
      adapter_(adapter),
      state_(state)
{
}

PoseOptimizationObjectiveFunction::~PoseOptimizationObjectiveFunction()
{
}

void PoseOptimizationObjectiveFunction::setStance(const Stance& stance)
{
  stance_ = stance;
}

const Stance& PoseOptimizationObjectiveFunction::getStance() const
{
  return stance_;
}

void PoseOptimizationObjectiveFunction::setNominalStance(
    const Stance& nominalStanceInBaseFrame)
{
  nominalStanceInBaseFrame_ = nominalStanceInBaseFrame;
}

const Stance& PoseOptimizationObjectiveFunction::getNominalStance() const
{
  return nominalStanceInBaseFrame_;
}

void PoseOptimizationObjectiveFunction::setInitialPose(const Pose& pose)
{
  initialPose_ = pose;
}

bool PoseOptimizationObjectiveFunction::computeValue(numopt_common::Scalar& value,
                                                     const numopt_common::Parameterization& params,
                                                     bool newParams)
{
  auto& poseParameterization = dynamic_cast<const PoseParameterization&>(params);
  State state(state_);
  state.setPoseBaseToWorld(poseParameterization.getPose());
  adapter_.setInternalDataFromState(state);
  value = 0.0;

  // Cost for default leg positions.
  for (const auto& footPosition : stance_) {
    const Position baseToFootInBase = adapter_.transformPosition(
        adapter_.getWorldFrameId(), adapter_.getBaseFrameId(), footPosition.second);
    const auto baseToHipInBase = adapter_.getPositionBaseToHipInBaseFrame(footPosition.first);
    const Vector hipToFootInBase(baseToFootInBase - baseToHipInBase);
    const Eigen::Array3d weight(1.0, 1.0, 10.0);
    value += (weight * (baseToFootInBase - nominalStanceInBaseFrame_.at(footPosition.first)).vector().array()).matrix().squaredNorm();
  }

  // Cost for deviation from initial position.
  Vector positionDifference(state.getPositionWorldToBaseInWorldFrame() - initialPose_.getPosition());
  const Eigen::Array3d positionRegularizerWeights(0.1, 0.1, 0.1);
  value += (positionRegularizerWeights * positionDifference.vector().array()).matrix().squaredNorm();

  // Cost for deviation from initial orientation.
//  EulerAnglesZyx rotationDifference(state.getOrientationBaseToWorld() * initialPose_.getRotation().inverted());
  EulerAnglesZyx rotationDifference(state.getOrientationBaseToWorld());
  rotationDifference.setUnique();
  const Eigen::Array3d rotationRegularizerWeights(0.0, 5.0, 5.0);
  value += (rotationRegularizerWeights * rotationDifference.vector().array()).matrix().squaredNorm();

  adapter_.setInternalDataFromState(state_);
  return true;
}

bool PoseOptimizationObjectiveFunction::getLocalGradient(numopt_common::Vector& gradient,
                                                         const numopt_common::Parameterization& p, bool newParams)
{
  return NonlinearObjectiveFunction::estimateLocalGradient(gradient, p, 1.0e-6);
}

} /* namespace free_gait */

