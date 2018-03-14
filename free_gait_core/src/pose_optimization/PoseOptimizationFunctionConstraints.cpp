/*
 * PoseOptimizationFunctionConstraints.cpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#include "free_gait_core/pose_optimization/PoseOptimizationFunctionConstraints.hpp"
#include "free_gait_core/pose_optimization/PoseParameterization.hpp"

#include <stdexcept>

namespace free_gait {

PoseOptimizationFunctionConstraints::PoseOptimizationFunctionConstraints()
    : NonlinearFunctionConstraints(),
      nSupportRegionInequalityConstraints_(0),
      nLimbLengthInequalityConstraints_(0)
{
}

PoseOptimizationFunctionConstraints::~PoseOptimizationFunctionConstraints()
{
}

void PoseOptimizationFunctionConstraints::setStance(const Stance& stance)
{
  stance_ = stance;
}

void PoseOptimizationFunctionConstraints::setSupportRegion(const grid_map::Polygon& supportRegion)
{
  supportRegion_ = supportRegion;
  nSupportRegionInequalityConstraints_ = supportRegion.nVertices();
  updateNumberOfInequalityConstraints();
  supportRegion_.convertToInequalityConstraints(supportRegionInequalityConstraintGlobalJacobian_,
                                                supportRegionInequalityConstraintsMaxValues_);
}

void PoseOptimizationFunctionConstraints::setLimbLengthConstraints(
    const LimbLengths& minLimbLenghts,
    const LimbLengths& maxLimbLenghts)
{
  if (minLimbLenghts.size() != maxLimbLenghts.size()) {
    throw std::invalid_argument("[PoseOptimizationFunctionConstraints::setLimbLengthConstraints]"
        " minLimbLenghts and maxLimbLenghts need to have to same size!");
  }

  nLimbLengthInequalityConstraints_= minLimbLenghts.size();
  updateNumberOfInequalityConstraints();

//  if (nLimbLengthInequalityConstraints_ != adapter_.getLimbs().size()) {
//    throw std::invalid_argument("[PoseOptimizationFunctionConstraints::setLimbLengthConstraints]"
//        " if used, minLimbLenghts and maxLimbLenghts need to be specified for all legs!");
//  }

  limbLengthInequalityConstraintsMinValues_.resize(nLimbLengthInequalityConstraints_);
  limbLengthInequalityConstraintsMaxValues_.resize(nLimbLengthInequalityConstraints_);

  size_t i(0);
  for (const auto& minLimbLength : minLimbLenghts) {
    const auto limb = minLimbLength.first;
    limbLengthInequalityConstraintsMinValues_(i) = minLimbLenghts.at(limb);
    limbLengthInequalityConstraintsMaxValues_(i) = maxLimbLenghts.at(limb);
    ++i;
  }
}

void PoseOptimizationFunctionConstraints::setPositionsBaseToHip(
    const LegPositions& positionBaseToHipInBaseFrame)
{
  positionsBaseToHipInBaseFrame_ = positionBaseToHipInBaseFrame;
}

void PoseOptimizationFunctionConstraints::setCenterOfMass(const Position& centerOfMassInBaseFrame)
{
  centerOfMassInBaseFrame_ = centerOfMassInBaseFrame;
}

bool PoseOptimizationFunctionConstraints::getGlobalBoundConstraintMinValues(
    numopt_common::Vector& values)
{
  values = Eigen::VectorXd::Constant(PoseParameterization::getGlobalSizeStatic(), std::numeric_limits<double>::lowest());
  return true;
}

bool PoseOptimizationFunctionConstraints::getGlobalBoundConstraintMaxValues(
    numopt_common::Vector& values)
{
  values = Eigen::VectorXd::Constant(PoseParameterization::getGlobalSizeStatic(), std::numeric_limits<double>::max());
  return true;
}

bool PoseOptimizationFunctionConstraints::getInequalityConstraintValues(numopt_common::Vector& values,
                                                                        const numopt_common::Parameterization& p,
                                                                        bool newParams)
{
  values.resize(getNumberOfInequalityConstraints());

  const auto& poseParameterization = dynamic_cast<const PoseParameterization&>(p);
  const Pose basePose = poseParameterization.getPose();

  // Support region.
  const Position centerOfMassInWorldFrame = basePose.getPosition() + basePose.getRotation().rotate(centerOfMassInBaseFrame_);

  values.segment(0, nSupportRegionInequalityConstraints_) =
//      supportRegionInequalityConstraintGlobalJacobian_ * basePose.getPosition().vector().head(2);
      supportRegionInequalityConstraintGlobalJacobian_ * centerOfMassInWorldFrame.vector().head(2);

  // Leg length.
  size_t i(0);
  for (const auto& leg : stance_) {
    const auto limb = leg.first;
    const Position& footPosition = leg.second;
    const Position baseToFootInBase = basePose.getRotation().inverseRotate(footPosition - basePose.getPosition());
    const double legLength = Vector(baseToFootInBase - positionsBaseToHipInBaseFrame_.at(limb)).norm();
    values(nSupportRegionInequalityConstraints_ + i) = legLength;
    ++i;
  }

  return true;
}

bool PoseOptimizationFunctionConstraints::getInequalityConstraintMinValues(numopt_common::Vector& d)
{
  d = numopt_common::Vector::Constant(nInequalityConstraints_, std::numeric_limits<double>::lowest());

  // Leg length.
  d.segment(nSupportRegionInequalityConstraints_, nLimbLengthInequalityConstraints_) =
      limbLengthInequalityConstraintsMinValues_;

  return true;
}

bool PoseOptimizationFunctionConstraints::getInequalityConstraintMaxValues(numopt_common::Vector& f)
{
  f = numopt_common::Vector::Constant(nInequalityConstraints_, std::numeric_limits<double>::max());

  // Support region.
  f.segment(0, nSupportRegionInequalityConstraints_) = supportRegionInequalityConstraintsMaxValues_;

  // Leg length.
  f.segment(nSupportRegionInequalityConstraints_, nLimbLengthInequalityConstraints_) =
      limbLengthInequalityConstraintsMaxValues_;

  return true;
}

bool PoseOptimizationFunctionConstraints::getLocalInequalityConstraintJacobian(
    numopt_common::SparseMatrix& jacobian, const numopt_common::Parameterization& params, bool newParams)
{
  // Numerical approach.
//  numopt_common::SparseMatrix numericalJacobian(getNumberOfInequalityConstraints(), params.getLocalSize());
//  NonlinearFunctionConstraints::estimateLocalInequalityConstraintJacobian(numericalJacobian, params);
//  std::cout << "Numerical:\n" << numericalJacobian << std::endl;

  // Analytical approach.
  Eigen::MatrixXd analyticalJacobian(getNumberOfInequalityConstraints(), params.getLocalSize());
  const auto& poseParameterization = dynamic_cast<const PoseParameterization&>(params);
  const Pose pose = poseParameterization.getPose();
  const Eigen::Vector3d& p = pose.getPosition().vector();
  const RotationQuaternion Phi(pose.getRotation());
  const Eigen::Vector3d Phi_r_com = Phi.rotate(centerOfMassInBaseFrame_).vector();
  const Eigen::Matrix3d Phi_r_com_skew = kindr::getSkewMatrixFromVector(Phi_r_com);

  // Support region.
  Eigen::MatrixXd G_SP(nSupportRegionInequalityConstraints_, 3);
  G_SP.setZero();
  G_SP.leftCols(2) = supportRegionInequalityConstraintGlobalJacobian_;
  analyticalJacobian.topLeftCorner(nSupportRegionInequalityConstraints_, 3) = G_SP;
  analyticalJacobian.topRightCorner(nSupportRegionInequalityConstraints_, 3) = -G_SP * Phi_r_com_skew;

  // Leg length.
  size_t i(0);
  for (const auto& leg : stance_) {
    const auto limb = leg.first;
    const Position& footPosition = leg.second;
    const Eigen::Vector3d Phi_r_BH = Phi.rotate(positionsBaseToHipInBaseFrame_.at(limb)).vector();
    const Eigen::Matrix3d Phi_r_BH_skew = kindr::getSkewMatrixFromVector(Phi_r_BH);
    const Eigen::Vector3d l_normalized = (p + Phi_r_BH - footPosition.vector()).normalized();
    analyticalJacobian.row(nSupportRegionInequalityConstraints_ + i).head<3>() = l_normalized;
    analyticalJacobian.row(nSupportRegionInequalityConstraints_ + i).tail<3>() = -l_normalized.transpose() * Phi_r_BH_skew;
    ++i;
  }

//  std::cout << "Analytical:\n" << analyticalJacobian << std::endl;

  // Return solution.
//  jacobian = numericalJacobian;
  jacobian = analyticalJacobian.sparseView(1e-10);

  return true;
}


void PoseOptimizationFunctionConstraints::updateNumberOfInequalityConstraints()
{
  setNumberOfInequalityConstraints(
      nSupportRegionInequalityConstraints_ + nLimbLengthInequalityConstraints_);
}

} /* namespace */
