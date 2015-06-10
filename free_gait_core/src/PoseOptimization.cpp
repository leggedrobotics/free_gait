/*
 * PoseOptimization.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/PoseOptimization.hpp>
#include <Eigen/Core>
#include <ooqp_eigen_interface/OoqpEigenInterface.hpp>
#include <kindr/rotations/RotationEigen.hpp>

using namespace Eigen;

namespace free_gait {

PoseOptimization::PoseOptimization()
    : nStates_(3)
{
}

PoseOptimization::~PoseOptimization()
{
}

void PoseOptimization::setFeetPositions(const std::vector<Position>& feetPositions)
{
  feetPositions_ = feetPositions;
}

void PoseOptimization::setDesiredLegConfiguration(
    const std::vector<Position>& desiredFeetPositionsInBase)
{
  desiredFeetPositionsInBase_ = desiredFeetPositionsInBase;
}

void PoseOptimization::setSupportPolygon(const grid_map::Polygon& supportPolygon,
                                         const double safetyMargin)
{
  supportPolygon_ = supportPolygon;
  safetyMargin_ = safetyMargin;
}

void PoseOptimization::setStartPose(const Pose& startPose)
{
  startPose_ = startPose;
}

bool PoseOptimization::compute(Pose& optimizedPose)
{
  // Problem definition:
  // min Ax - b, Gx <= h
  unsigned int nFeet = feetPositions_.size();
  MatrixXd A = MatrixXd::Zero(2 * nFeet, nStates_);
  VectorXd b = VectorXd::Zero(2 * nFeet);
  Matrix3d R_0 = RotationMatrix(startPose_.getRotation().inverted()).matrix();
  Matrix3d Rstar;
  Rstar << 0, -1, 0,
           1,  0, 0,
           0,  0, 0;

  for (unsigned int i = 0; i < nFeet; i++) {
    A.block(2 * i, 0, 2, A.cols()) << Matrix2d::Identity(), (R_0 * Rstar * desiredFeetPositionsInBase_[i].vector()).head<2>();
    b.segment(2 * i, 2) << feetPositions_[i].vector() - R_0 * desiredFeetPositionsInBase_[i].vector();
  }

//  std::cout << "R_0: " << std::endl << R_0 << std::endl;
//  std::cout << "A: " << std::endl << A << std::endl;
//  std::cout << "b: " << std::endl << b << std::endl;

  // Inequality constraints.

  // Formulation as QP:
  // min 1/2 x'Px + q'x + r
  MatrixXd P = 2 * A.transpose() * A;
  VectorXd q = -2 * A.transpose() * b;
//  MatrixXd r = b.transpose() * b; // Not used.

  // Solve.
  Eigen::SparseMatrix<double, Eigen::RowMajor> P_sparse = P.sparseView();
  Eigen::VectorXd x;
  if (!ooqpei::OoqpEigenInterface::solve(P_sparse, q, x))
      return false;
//  std::cout << "x: " << std::endl << x << std::endl;

  // Return optimized pose.
  optimizedPose.getPosition() << x.head<2>(), startPose_.getPosition().z();
  const double yaw = x.tail<1>()[0];
  optimizedPose.getRotation() = (RotationMatrix(R_0) * kindr::rotations::eigen_impl::EulerAnglesZyxPD(yaw, 0.0, 0.0)).inverted();

  return true;
}

} /* namespace */
