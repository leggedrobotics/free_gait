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

  // Cost for foot below hip.
//  for (const auto& foot : stance_) {
//    const Position footPosition(foot.second);
//    const Position hipInWorld = pose.getPosition() + pose.getRotation().rotate(positionsBaseToHipInBaseFrame_.at(foot.first));
//    value += (footPosition - hipInWorld).vector().head(2).squaredNorm();
//  }

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
  value += 0.5 * (supportRegion_.getCentroid().head(2) - centerOfMassInWorldFrame.vector().head(2)).squaredNorm();

//  // Cost for torques.
////  state_.setPoseBaseToWorld(pose);
////  adapter_.setInternalDataFromState(state_); // To guide IK.
////
////  for (const auto& foot : stance_) {
////    const Position footPositionInBase(
////        adapter_.transformPosition(adapter_.getWorldFrameId(), adapter_.getBaseFrameId(), foot.second));
////    JointPositionsLeg jointPositions;
////    const bool success = adapter_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(footPositionInBase, foot.first, jointPositions);
////    if (success) state_.setJointPositionsForLimb(foot.first, jointPositions);
////  }
//
////  adapter_.setInternalDataFromState(state_);
//
////  adapter_.
//
//  for (const auto& foot : stance_) {
//    const Position footPosition(foot.second);
//    const Position baseToFootInBase = pose.getRotation().inverseRotate(footPosition - pose.getPosition());
//    Eigen::Vector3d jointPositions;
//    // https://ashwinnarayan.blogspot.ch/2014/07/inverse-kinematics-for-2dof-arm.html
//    const double l1 = 1.0;
//    const double l1_2 = l1 * l1;
//    const double l2 = 1.0;
//    const double l2_2 = l2 * l2;
//
//    const double x = -baseToFootInBase(2);
//    const double x2 = x * x;
//    const double y = baseToFootInBase(0);
//    const double y2 = y * y;
//    const double k = (x2 + y2 - l1_2 - l2_2) / (2 * l1 * l2);
//    jointPositions(0) = atan2(sqrt(1 - k * k), k);
//
//    const double k1 = l1 + l2 * cos(jointPositions(1));
//    const double k2 = l2 * sin(jointPositions(1));
//    const double gamma = atan2(k2, k1);
//    jointPositions(1) = atan2(y, x) - gamma;
//    value += 0.1 * jointPositions.array().sin().matrix().squaredNorm();
////    JointPositionsLeg jointPositions;
////    adapter_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(baseToFootInBase, foot.first, jointPositions);
////    value += 0.01 * jointPositions.vector().array().sin().matrix().squaredNorm();
//  }


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

