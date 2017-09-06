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

PoseOptimizationObjectiveFunction::PoseOptimizationObjectiveFunction()
    : NonlinearObjectiveFunction()
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

void PoseOptimizationObjectiveFunction::setSupportRegion(const grid_map::Polygon& supportRegion)
{
  supportRegion_ = supportRegion;
}


void PoseOptimizationObjectiveFunction::setCenterOfMass(const Position& centerOfMassInBaseFrame)
{
  centerOfMassInBaseFrame_ = centerOfMassInBaseFrame;
}

bool PoseOptimizationObjectiveFunction::computeValue(numopt_common::Scalar& value,
                                                     const numopt_common::Parameterization& params,
                                                     bool newParams)
{
  const auto& poseParameterization = dynamic_cast<const PoseParameterization&>(params);
  const Pose pose = poseParameterization.getPose();
  value = 0.0;

  // Cost for default leg positions.
  for (const auto& footPosition : stance_) {
    const Position nominalFootPositionInWorld = pose.getPosition()
        + pose.getRotation().rotate(nominalStanceInBaseFrame_.at(footPosition.first)); // d_i
    value += (nominalFootPositionInWorld - footPosition.second).squaredNorm();
  }

//  // Cost for deviation from initial position.
//  Vector positionDifference(state.getPositionWorldToBaseInWorldFrame() - initialPose_.getPosition());
//  value += 0.1 * positionDifference.vector().squaredNorm();

  // Cost for deviation from initial orientation.
//  RotationQuaternion rotationDifference(pose.getRotation() * initialPose_.getRotation().inverted());
//  const double rotationDifferenceNorm = rotationDifference.norm();
//  value += 10.0 * rotationDifferenceNorm * rotationDifferenceNorm;

  // Cost for deviation from horizontal pose.
//  RotationVector rotationDifference(pose.getRotation());
//  rotationDifference.toImplementation().z() = 0.0;
//  const double rotationDifferenceNorm = rotationDifference.vector().norm();
//  value += 1.0 * rotationDifferenceNorm;

  // Cost for CoM.
  const Position centerOfMassInWorldFrame = pose.getPosition() + pose.getRotation().rotate(centerOfMassInBaseFrame_);
  value += 4.0 * (supportRegion_.getCentroid().head(2) - centerOfMassInWorldFrame.vector().head(2)).squaredNorm();

  return true;
}

bool PoseOptimizationObjectiveFunction::getLocalGradient(numopt_common::Vector& gradient,
                                                         const numopt_common::Parameterization& params, bool newParams)
{
  return NonlinearObjectiveFunction::estimateLocalGradient(gradient, params, 1.0e-6);

//  const auto& poseParameterization = dynamic_cast<const PoseParameterization&>(params);
//  const Pose pose = poseParameterization.getPose();
//  gradient.setZero();
//
//  const Eigen::Vector3d& p = pose.getPosition().vector();
//  const auto& R = pose.getRotation(); // Phi
////  const Position centerOfMassInWorldFrame = pose.getPosition() + pose.getRotation().rotate(centerOfMassInBaseFrame_);
////  const auto& r_CoM = centerOfMassInBaseFrame_.vector();
//  for (const auto& footPosition : stance_) {
//    const Eigen::Vector3d f_i = footPosition.second.vector();
//    const Eigen::Vector3d R_d_i = pose.getRotation().rotate(nominalStanceInBaseFrame_.at(footPosition.first)).vector();
//    Eigen::VectorXd derivative(params.getLocalSize());
//    derivative << p + R_d_i - f_i, -p.array() * R_d_i.array() + R_d_i.array() * f_i.array();
//    gradient += derivative;
//    std::cout << derivative << std::endl << std::endl;
//  }
//  gradient = 2.0 * gradient; // w_1 = 1.0;
//  std::cout << gradient << std::endl << std::endl;
//  std::cout << "++++++++++" << std::endl;
//  return true;
}

} /* namespace free_gait */

