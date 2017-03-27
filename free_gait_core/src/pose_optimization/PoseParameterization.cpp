/*
 * PoseParameterization.cpp
 *
 *  Created on: Mar 22, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "free_gait_core/pose_optimization/PoseParameterization.hpp"

namespace free_gait {

PoseParameterization::PoseParameterization()
{
  setIdentity(params_);
}

PoseParameterization::~PoseParameterization()
{
}

//PoseParameterization::PoseParameterization(const PoseParameterization& other)
//    : params_(other.params_)
//{
//}

numopt_common::Params& PoseParameterization::getParams()
{
  return params_;
}

const numopt_common::Params& PoseParameterization::getParams() const
{
  return params_;
}

bool PoseParameterization::plus(numopt_common::Params& result, const numopt_common::Params& p,
                                const numopt_common::Delta& dp) const
{
  // Position.
  result.head(nTransGlobal_) = p.head(nTransGlobal_) + dp.head(nTransLocal_);

  // Orientation.
  result.tail(nRotGlobal_) = RotationQuaternion(p.tail(nRotGlobal_)).boxPlus(
      dp.tail(nRotLocal_)).vector();

  return true;
}

bool PoseParameterization::getTransformMatrixLocalToGlobal(
    numopt_common::SparseMatrix& matrix, const numopt_common::Params& params) const
{
  Eigen::MatrixXd denseMatrix(Eigen::MatrixXd::Zero(getGlobalSize(), getLocalSize()));
  denseMatrix.topLeftCorner(nTransGlobal_, nTransLocal_).setIdentity();
  denseMatrix.bottomRightCorner(nRotGlobal_, nRotLocal_) =
      0.5 * RotationQuaternion(params.tail(nRotGlobal_))
           .getLocalQuaternionDiffMatrix().transpose();
  matrix = denseMatrix.sparseView();
  return true;
}

bool PoseParameterization::getTransformMatrixGlobalToLocal(
    numopt_common::SparseMatrix& matrix, const numopt_common::Params& params) const
{
  Eigen::MatrixXd denseMatrix(Eigen::MatrixXd::Zero(getLocalSize(), getGlobalSize()));
  denseMatrix.topLeftCorner(nTransLocal_, nTransGlobal_).setIdentity();
  denseMatrix.bottomRightCorner(nRotLocal_, nRotGlobal_) = 2.0
      * RotationQuaternion(params).getLocalQuaternionDiffMatrix();
  matrix = denseMatrix.sparseView();
  return true;
}

int PoseParameterization::getGlobalSize() const
{
  return nTransGlobal_ + nRotGlobal_;
}

const size_t PoseParameterization::getGlobalSizeStatic()
{
  return nTransGlobal_ + nRotGlobal_;
}

int PoseParameterization::getLocalSize() const
{
  return nTransLocal_ + nRotLocal_;
}

bool PoseParameterization::setRandom(numopt_common::Params& p) const
{
  p.resize(getGlobalSize());
  p.head(nTransGlobal_).setRandom();
  RotationQuaternion randomQuaternion;
  randomQuaternion.setRandom();
  p.tail(nRotGlobal_) = randomQuaternion.vector();
  return true;
}

bool PoseParameterization::setIdentity(numopt_common::Params& p) const
{
  p.resize(getGlobalSize());
  p.head(nTransGlobal_).setZero();
  RotationQuaternion identityQuaternion;
  identityQuaternion.setIdentity();
  p.tail(nRotGlobal_) = identityQuaternion.vector();
  return true;
}

numopt_common::Parameterization* PoseParameterization::clone() const
{
  Parameterization* clone = new PoseParameterization(*this);
  return clone;
}

const Pose PoseParameterization::getPose() const
{
  return Pose(getPosition(), getOrientation());
}

void PoseParameterization::setPose(const Pose& pose)
{
  params_.head(nTransGlobal_) = pose.getPosition().vector();
  params_.tail(nRotGlobal_) = pose.getRotation().vector();
}

const Position PoseParameterization::getPosition() const
{
  return Position(params_.head(nTransGlobal_));
}

const RotationQuaternion PoseParameterization::getOrientation() const
{
  return RotationQuaternion(params_.tail(nRotGlobal_));
}

} /* namespace */
