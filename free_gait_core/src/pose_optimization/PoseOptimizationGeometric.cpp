/*
 * PoseOptimizationGeometric.cpp
 *
 *  Created on: Aug 30, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#include <free_gait_core/pose_optimization/PoseOptimizationGeometric.hpp>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <kindr/Core>

using namespace Eigen;

namespace free_gait {

PoseOptimizationGeometric::PoseOptimizationGeometric(const AdapterBase& adapter)
    : PoseOptimizationBase(adapter)
{
}

PoseOptimizationGeometric::~PoseOptimizationGeometric()
{
}

void PoseOptimizationGeometric::setStanceForOrientation(const Stance& stance)
{
  stanceForOrientation_ = stance;
}

bool PoseOptimizationGeometric::optimize(Pose& pose)
{
  checkSupportRegion();

  // Planar position: Use geometric center of support region.
  Position center;
  center.vector().head(2) = supportRegion_.getCentroid();

  // Height: Average of stance plus default height.
  for (const auto& foot : stance_) {
    center.z() += foot.second.z() - nominalStanceInBaseFrame_.at(foot.first).z();
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
    kindr::QuaternionD footPositionInertialFrame(0.0, foot.second.vector()); // \bar_a_k = S^T * a_k (39).
    kindr::QuaternionD defaultFootPositionBaseFrame(0.0, nominalStanceInBaseFrame_.at(foot.first).vector()); // \bar_b_k = S^T * b_k.
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

  // Get yaw rotation desired heading direction with respect to the target feet.
  const Position positionForeFeetMidPointInWorld = (stanceForOrientation_.at(LimbEnum::LF_LEG) + stanceForOrientation_.at(LimbEnum::RF_LEG)) * 0.5;
  const Position positionHindFeetMidPointInWorld = (stanceForOrientation_.at(LimbEnum::LH_LEG) + stanceForOrientation_.at(LimbEnum::RH_LEG)) * 0.5;
  Vector desiredHeadingDirectionInWorld = Vector(positionForeFeetMidPointInWorld - positionHindFeetMidPointInWorld);
  desiredHeadingDirectionInWorld.z() = 0.0;
  RotationQuaternion desiredHeading;
  desiredHeading.setFromVectors(Vector::UnitX().toImplementation(), desiredHeadingDirectionInWorld.vector());

  // Apply roll/pitch adaptation factor (~0.7).
  const RotationQuaternion yawRotation(RotationVector(RotationVector(pose.getRotation()).vector().cwiseProduct(Eigen::Vector3d::UnitZ())));
  const RotationQuaternion rollPitchRotation(RotationVector(0.7 * RotationVector(yawRotation.inverted() * pose.getRotation()).vector()));
//  pose.getRotation() = yawRotation * rollPitchRotation; // Alternative.
  pose.getRotation() = desiredHeading * rollPitchRotation;

  return true;
}

} /* namespace */
