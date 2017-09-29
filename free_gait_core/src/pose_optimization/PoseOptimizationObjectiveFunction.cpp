/*
 * PoseOptimizationObjectiveFunction.cpp
 *
 *  Created on: Mar 22, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#include "free_gait_core/pose_optimization/PoseOptimizationObjectiveFunction.hpp"
#include "free_gait_core/pose_optimization/PoseParameterization.hpp"
#include "free_gait_core/TypeDefs.hpp"

namespace free_gait {

PoseOptimizationObjectiveFunction::PoseOptimizationObjectiveFunction()
    : NonlinearObjectiveFunction(),
      comWeight_(2.0)
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
  value += comWeight_ * (supportRegion_.getCentroid().head(2) - centerOfMassInWorldFrame.vector().head(2)).squaredNorm();

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
  // Numercical approach.
//  numopt_common::Vector numericalGradient(params.getLocalSize());
//  NonlinearObjectiveFunction::estimateLocalGradient(numericalGradient, params, 1.0e-6);
//  std::cout << "Numerical: " << numericalGradient.transpose() << std::endl;

  // Analytical approach.
  numopt_common::Vector analyticalGradient(params.getLocalSize());
  analyticalGradient.setZero();
  const auto& poseParameterization = dynamic_cast<const PoseParameterization&>(params);
  const Pose pose = poseParameterization.getPose();
  const Eigen::Vector3d& p = pose.getPosition().vector();

  // Default leg position.
  for (const auto& footPosition : stance_) {
    const Eigen::Vector3d f_i = footPosition.second.vector();
    const Eigen::Vector3d R_d_i = pose.getRotation().rotate(nominalStanceInBaseFrame_.at(footPosition.first)).vector();
    const Eigen::Matrix3d R_d_i_skew = kindr::getSkewMatrixFromVector(R_d_i);
    analyticalGradient.head(3) += p + R_d_i - f_i;
    analyticalGradient.tail(3) += R_d_i_skew * p - R_d_i_skew * f_i;
  }

  // Center of mass.
  const Eigen::Vector3d p_2d(p.x(), p.y(), 0.0); // Projection.
  Eigen::Vector3d R_r_com = pose.getRotation().rotate(centerOfMassInBaseFrame_).vector();
  R_r_com.z() = 0.0;
  const Eigen::Matrix3d R_r_com_skew = kindr::getSkewMatrixFromVector(R_r_com);
  const grid_map::Position supportPolygonCentroid(supportRegion_.getCentroid());
  const Eigen::Vector3d r_centroid(supportPolygonCentroid.x(), supportPolygonCentroid.y(), 0.0);
  analyticalGradient.head(3) += comWeight_ * (p_2d - r_centroid + R_r_com);
  analyticalGradient.tail(3) += comWeight_ * (R_r_com_skew * p_2d - R_r_com_skew * r_centroid);

  // Factorized with 2.0 (not weight!).
  analyticalGradient = 2.0 * analyticalGradient; // w_1 = 1.0;
//  std::cout << "Analytical: " << analyticalGradient.transpose() << std::endl << std::endl;

  // Return solution.
//  gradient = numericalGradient;
  gradient = analyticalGradient;
  return true;
}

bool PoseOptimizationObjectiveFunction::getLocalHessian(numopt_common::SparseMatrix& hessian,
                                                        const numopt_common::Parameterization& params, bool newParams)
{
  // Numerical approach.
//  numopt_common::SparseMatrix numericalHessian(params.getLocalSize(), params.getLocalSize());
//  NonlinearObjectiveFunction::estimateLocalHessian(numericalHessian, params, 1.0e-6);
//  std::cout << "Numerical:\n" << numericalHessian << std::endl;

  // Analytical approach.
  Eigen::MatrixXd analyticalHessian(params.getLocalSize(), params.getLocalSize());
  analyticalHessian.setZero();
  const auto& poseParameterization = dynamic_cast<const PoseParameterization&>(params);
  const Pose pose = poseParameterization.getPose();
  const Eigen::Vector3d& p = pose.getPosition().vector();

  // Default leg position.
  for (const auto& footPosition : stance_) {
    const Eigen::Vector3d f_i = footPosition.second.vector();
    const Eigen::Vector3d R_d_i = pose.getRotation().rotate(nominalStanceInBaseFrame_.at(footPosition.first)).vector();
    const Eigen::Matrix3d R_d_i_skew = kindr::getSkewMatrixFromVector(R_d_i);
    analyticalHessian.topLeftCorner(3, 3) += Eigen::Matrix3d::Identity();
    analyticalHessian.topRightCorner(3, 3) += -R_d_i_skew;
    analyticalHessian.bottomLeftCorner(3, 3) += R_d_i_skew;
    analyticalHessian.bottomRightCorner(3, 3) +=   kindr::getSkewMatrixFromVector(p) * R_d_i_skew
                                                 - kindr::getSkewMatrixFromVector(f_i) * R_d_i_skew;
  }

  // Center of mass.
  const Eigen::Vector3d p_2d(p.x(), p.y(), 0.0); // Projection.
  Eigen::Vector3d R_r_com = pose.getRotation().rotate(centerOfMassInBaseFrame_).vector();
  R_r_com.z() = 0.0;
  const Eigen::Matrix3d R_r_com_skew = kindr::getSkewMatrixFromVector(R_r_com);
  const grid_map::Position supportPolygonCentroid(supportRegion_.getCentroid());
  const Eigen::Vector3d r_centroid(supportPolygonCentroid.x(), supportPolygonCentroid.y(), 0.0);
  analyticalHessian.topLeftCorner(3, 3) += comWeight_ * Eigen::Vector3d(1.0, 1.0, 0.0).asDiagonal();
  analyticalHessian.topRightCorner(3, 3) += -comWeight_ * R_r_com_skew;
  analyticalHessian.bottomLeftCorner(3, 3) += comWeight_ * R_r_com_skew;
  analyticalHessian.bottomRightCorner(3, 3) += comWeight_ * (kindr::getSkewMatrixFromVector(p) * R_r_com_skew
                                               - kindr::getSkewMatrixFromVector(r_centroid) * R_r_com_skew);

  // Factorized with 2.0 (not weight!).
  analyticalHessian = 2.0 * analyticalHessian;
//  std::cout << "Analytical:\n" << analyticalHessian << std::endl << std::endl;

  // Return solution.
//  hessian = numericalHessian;
  hessian = analyticalHessian.sparseView(1e-10);
  return true;
}


} /* namespace free_gait */

