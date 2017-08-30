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
#include <kindr/Core>

using namespace Eigen;

namespace free_gait {

PoseOptimizationGeometric::PoseOptimizationGeometric()
{
}

PoseOptimizationGeometric::~PoseOptimizationGeometric()
{
}

const Pose PoseOptimizationGeometric::optimize(const Stance& stance, const Stance& nominalStanceInBaseFrame,
                                               const grid_map::Polygon& supportRegion)
{
  Pose pose;

  // If no support polygon provided, use positions.
  grid_map::Polygon supportRegionCopy(supportRegion);
  if (supportRegionCopy.nVertices() == 0) {
    for (const auto& foot : stance)
      supportRegionCopy.addVertex(foot.second.vector().head<2>());
  }

  // Planar position: Use geometric center of support region.
  Position center;
  center.vector().head(2) = supportRegionCopy.getCentroid();

  // Height: Average of stance plus default height.
  for (const auto& foot : stance) {
    center.z() += foot.second.z() - nominalStanceInBaseFrame.at(foot.first).z();
  }
  center.z() /= (double) stance.size();
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
  for (const auto& foot : stance) {
    const RotationQuaternion footPositionInertialFrame(0.0, foot.second.vector()); // \bar_a_k = S^T * a_k (39).
    const RotationQuaternion defaultFootPositionBaseFrame(0.0, nominalStanceInBaseFrame.at(foot.first).vector()); // \bar_b_k = S^T * b_k.
    Eigen::Matrix4d Ak = footPositionInertialFrame.getQuaternionMatrix() - defaultFootPositionBaseFrame.getConjugateQuaternionMatrix(); // See (46).
    C += Ak * Ak; // See (45).
    A += Ak; // See (46).
  }
  A = A / ((double) stance.size()); // Error in (46).
  C -= stance.size() * A * A; // See (45).
  Eigen::EigenSolver<Eigen::Matrix4d> eigenSolver(C);
  int maxCoeff;
  eigenSolver.eigenvalues().real().maxCoeff(&maxCoeff);
  // Eigen vector corresponding to max. eigen value.
  pose.getRotation() = RotationQuaternion(eigenSolver.eigenvectors().col(maxCoeff).real());
  pose.getRotation().setUnique();

  // Apply roll/pitch adaptation factor (~0.7).
  const RotationQuaternion yawRotation(RotationVector(RotationVector(pose.getRotation()).vector().cwiseProduct(Eigen::Vector3d::UnitZ())));
  const RotationQuaternion rollPitchRotation(RotationVector(0.7 * RotationVector(pose.getRotation() * yawRotation.inverted()).vector()));
  pose.getRotation() = yawRotation * rollPitchRotation;

  return pose;
}

} /* namespace */
