/*
 * PoseOptimizationQP.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/pose_optimization/PoseOptimizationQP.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <kindr/Core>
#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>

using namespace Eigen;

namespace free_gait {

PoseOptimizationQP::PoseOptimizationQP()
    : nStates_(4),
      nDimensions_(3)
{
  solver_.reset(new numopt_quadprog::ActiveSetFunctionMinimizer());
}

PoseOptimizationQP::~PoseOptimizationQP()
{
}

PoseOptimizationQP::PoseOptimizationQP(const PoseOptimizationQP& other)
    : nStates_(other.nStates_),
      nDimensions_(other.nDimensions_),
      stance_(other.stance_),
      nominalStanceInBaseFrame_(other.nominalStanceInBaseFrame_),
      supportRegion_(other.supportRegion_)
{
  solver_.reset(new numopt_quadprog::ActiveSetFunctionMinimizer());
}

void PoseOptimizationQP::setStance(const Stance& stance)
{
  stance_ = stance;
}

void PoseOptimizationQP::setNominalStance(
    const Stance& nominalStanceInBaseFrame)
{
  nominalStanceInBaseFrame_ = nominalStanceInBaseFrame;
}

void PoseOptimizationQP::setSupportRegion(const grid_map::Polygon& supportRegion)
{
  supportRegion_ = supportRegion;
}

bool PoseOptimizationQP::optimize(Pose& pose)
{
  // If no support polygon provided, use positions.
  if (supportRegion_.nVertices() == 0) {
    for (const auto& foot : stance_)
      supportRegion_.addVertex(foot.second.vector().head<2>());
  }

  // Problem definition:
  // min Ax - b, Gx <= h
  unsigned int nFeet = stance_.size();
  MatrixXd A = MatrixXd::Zero(nDimensions_ * nFeet, nStates_);
  VectorXd b = VectorXd::Zero(nDimensions_ * nFeet);
  Matrix3d R_0 = RotationMatrix(pose.getRotation()).matrix();
  Matrix3d Rstar;
  Rstar << 0.0, -1.0, 0.0,
           1.0,  0.0, 0.0,
           0.0,  0.0, 0.0;

  unsigned int i = 0;
  for (const auto& footPosition : stance_) {
    A.block(nDimensions_ * i, 0, nStates_-1, A.cols()) << Matrix3d::Identity(), (R_0 * Rstar * nominalStanceInBaseFrame_[footPosition.first].vector());
    b.segment(nDimensions_ * i, nDimensions_) << footPosition.second.vector() - R_0 * nominalStanceInBaseFrame_[footPosition.first].vector();
    ++i;
  }

//  std::cout << "R_0: " << std::endl << R_0 << std::endl;
//  std::cout << "A: " << std::endl << A << std::endl;
//  std::cout << "b: " << std::endl << b << std::endl;

  // Inequality constraints.
  Eigen::MatrixXd Gp;
  Eigen::VectorXd h;
  supportRegion_.convertToInequalityConstraints(Gp, h);
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

  // Cost function.
  auto costFunction = std::shared_ptr<numopt_common::QuadraticObjectiveFunction>(new numopt_common::QuadraticObjectiveFunction());
  numopt_common::SparseMatrix P_sparse = P.sparseView();
  costFunction->setGlobalHessian(P_sparse);
  costFunction->setLinearTerm(q);

  // Constraints.
  auto constraints = std::shared_ptr<numopt_common::LinearFunctionConstraints>(new numopt_common::LinearFunctionConstraints());
  numopt_common::SparseMatrix G_sparse = G.sparseView();
  constraints->setGlobalInequalityConstraintJacobian(G_sparse);
  constraints->setInequalityConstraintMinValues(std::numeric_limits<double>::lowest() * numopt_common::Vector::Ones(h.size()));
  constraints->setInequalityConstraintMaxValues(h);

  // Solve.
  numopt_common::QuadraticProblem problem(costFunction, constraints);

  Eigen::VectorXd x;
  numopt_common::ParameterizationIdentity params(x.size());
  params.getParams() = x;
  double cost = 0.0;
  if (!solver_->minimize(&problem, params, cost)) return false;
  x = params.getParams();
//  std::cout << "x: " << std::endl << x << std::endl;

  // Return optimized pose.
  pose.getPosition().vector() = x.head<3>();
  const double yaw = x.tail<1>()[0];
  Eigen::Matrix3d rotationMatrix((R_0 * (Matrix3d::Identity() + Rstar * yaw)));

  // http://people.csail.mit.edu/bkph/articles/Nearest_Orthonormal_Matrix.pdf
  // as discussed in https://github.com/ethz-asl/kindr/issues/55 ,
  // code by Philipp Krüsi.
  // TODO: Move to kindr.
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(rotationMatrix, Eigen::ComputeFullV);
  Eigen::Matrix3d correction(
      svd.matrixV().col(0) * svd.matrixV().col(0).transpose() /
      svd.singularValues()(0) +
      svd.matrixV().col(1) * svd.matrixV().col(1).transpose() /
      svd.singularValues()(1) +
      svd.matrixV().col(2) * svd.matrixV().col(2).transpose() /
      svd.singularValues()(2));

  pose.getRotation() = RotationMatrix(rotationMatrix * correction);
  return true;
}

} /* namespace */
