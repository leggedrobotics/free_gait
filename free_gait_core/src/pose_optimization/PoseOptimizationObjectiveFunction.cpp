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
    const Position nominalFootPositionInWorld = adapter_.getPositionWorldToBaseInWorldFrame()
        + adapter_.getOrientationBaseToWorld().rotate(nominalStanceInBaseFrame_.at(footPosition.first));
    value += (nominalFootPositionInWorld - footPosition.second).squaredNorm();
  }

//  // Cost for deviation from initial position.
//  Vector positionDifference(state.getPositionWorldToBaseInWorldFrame() - initialPose_.getPosition());
//  value += 0.1 * positionDifference.vector().squaredNorm();

  // Cost for deviation from initial orientation.
  RotationQuaternion rotationDifference(state.getOrientationBaseToWorld() * initialPose_.getRotation().inverted());
  const double rotationDifferenceNorm = rotationDifference.norm();
  value += 5.0 * rotationDifferenceNorm * rotationDifferenceNorm;

  adapter_.setInternalDataFromState(state_);
  return true;
}

bool PoseOptimizationObjectiveFunction::getLocalGradient(numopt_common::Vector& gradient,
                                                         const numopt_common::Parameterization& p, bool newParams)
{
  return NonlinearObjectiveFunction::estimateLocalGradient(gradient, p, 1.0e-6);
}

} /* namespace free_gait */

