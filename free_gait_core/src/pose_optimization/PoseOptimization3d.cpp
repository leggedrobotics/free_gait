/*
 * PoseOptimization.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/pose_optimization/PoseOptimization.hpp>
#include <Eigen/Core>
#include <ooqp_eigen_interface/OoqpEigenInterface.hpp>
#include <kindr/rotations/RotationEigen.hpp>

using namespace Eigen;

namespace free_gait {

PoseOptimization::PoseOptimization()
    : nStates_(4),
      nDimensions_(3)
{
}

PoseOptimization::~PoseOptimization()
{
}

void PoseOptimization::setFeetPositions(const FeetPositions& feetPositions)
{
  feetPositions_ = feetPositions;
}

void PoseOptimization::setDesiredLegConfiguration(
    const FeetPositions& desiredFeetPositionsInBase)
{
  desiredFeetPositionsInBase_ = desiredFeetPositionsInBase;
}

void PoseOptimization::setSupportPolygon(const grid_map::Polygon& supportPolygon)
{
  supportPolygon_ = supportPolygon;
}

bool PoseOptimization::optimize(Pose& pose)
{
  // If no support polygon provided, use positions.
  if (supportPolygon_.nVertices() == 0) {
    for (const auto& foot : feetPositions_)
      supportPolygon_.addVertex(foot.second.vector().head<2>());
  }

  // Problem definition:
  // min Ax - b, Gx <= h
  unsigned int nFeet = feetPositions_.size();
  MatrixXd A = MatrixXd::Zero(nDimensions_ * nFeet, nStates_);
  VectorXd b = VectorXd::Zero(nDimensions_ * nFeet);
  Matrix3d R_0 = RotationMatrix(pose.getRotation().inverted()).matrix();
  Matrix3d Rstar;
  Rstar << 0, -1, 0,
           1,  0, 0,
           0,  0, 0;

  unsigned int i = 0;
  for (const auto& footPosition : feetPositions_) {
    A.block(nDimensions_ * i, 0, nStates_-1, A.cols()) << Matrix3d::Identity(), (R_0 * Rstar * desiredFeetPositionsInBase_[footPosition.first].vector());
    b.segment(nDimensions_ * i, nDimensions_) << footPosition.second.vector() - R_0 * desiredFeetPositionsInBase_[footPosition.first].vector();
    ++i;
  }

//  std::cout << "R_0: " << std::endl << R_0 << std::endl;
//  std::cout << "A: " << std::endl << A << std::endl;
//  std::cout << "b: " << std::endl << b << std::endl;

  // Inequality constraints.
  Eigen::MatrixXd Gp;
  Eigen::VectorXd h;
  supportPolygon_.convertToInequalityConstraints(Gp, h);
  Eigen::MatrixXd G(Gp.rows(), nStates_);
  G << Gp, Eigen::MatrixXd::Zero(Gp.rows(), nStates_ - Gp.cols());

//  std::cout << "Gp: " << std::endl << Gp << std::endl;
//  std::cout << "G: " << std::endl << G << std::endl;
//  std::cout << "h: " << std::endl << h << std::endl;

  // Formulation as QP:
  // min 1/2 x'Px + q'x + r
  MatrixXd P = 2 * A.transpose() * A;
  VectorXd q = -2 * A.transpose() * b;
//  MatrixXd r = b.transpose() * b; // Not used.

  // Solve.
  Eigen::SparseMatrix<double, Eigen::RowMajor> P_sparse = P.sparseView();
  Eigen::SparseMatrix<double, Eigen::RowMajor> G_sparse = G.sparseView();
  Eigen::VectorXd x;
  if (!ooqpei::OoqpEigenInterface::solve(P_sparse, q, G_sparse, h, x))
      return false;
//  std::cout << "x: " << std::endl << x << std::endl;

  // Return optimized pose.
  pose.getPosition().vector() = x;
  const double yaw = x.tail<1>()[0];
  pose.getRotation() = RotationMatrix(R_0 * (Matrix3d::Identity() + Rstar * yaw)).transpose();
  pose.getRotation().fix();

  return true;
}

} /* namespace */
