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

PoseOptimizationFunctionConstraints::PoseOptimizationFunctionConstraints(const AdapterBase& adapter)
    : NonlinearFunctionConstraints(),
      adapter_(adapter),
      nSupportRegionInequalityConstraints_(0),
      nLimbLengthInequalityConstraints_(0)
{
  LegLengths minLimbLenghts, maxLimbLenghts;
  for (const auto& limb : adapter_.getLimbs()) {
    minLimbLenghts[limb] = 0.15;
    maxLimbLenghts[limb] = 0.52;
  }
  setLimbLengthConstraints(minLimbLenghts, maxLimbLenghts);
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

const grid_map::Polygon& PoseOptimizationFunctionConstraints::getSupportRegion() const
{
  return supportRegion_;
}

void PoseOptimizationFunctionConstraints::setLimbLengthConstraints(
    const LegLengths& minLimbLenghts,
    const LegLengths& maxLimbLenghts)
{
  if (minLimbLenghts.size() != maxLimbLenghts.size()) {
    throw std::invalid_argument("[PoseOptimizationFunctionConstraints::setLimbLengthConstraints]"
        " minLimbLenghts and maxLimbLenghts need to have to same size!");
  }

  nLimbLengthInequalityConstraints_= minLimbLenghts.size();
  updateNumberOfInequalityConstraints();

  if (nLimbLengthInequalityConstraints_ != adapter_.getLimbs().size()) {
    throw std::invalid_argument("[PoseOptimizationFunctionConstraints::setLimbLengthConstraints]"
        " if used, minLimbLenghts and maxLimbLenghts need to be specified for all legs!");
  }

  limbLengthInequalityConstraintsMinValues_.resize(nLimbLengthInequalityConstraints_);
  limbLengthInequalityConstraintsMaxValues_.resize(nLimbLengthInequalityConstraints_);

  size_t i(0);
  for (const auto& limb : adapter_.getLimbs()) {
    limbLengthInequalityConstraintsMinValues_(i) = minLimbLenghts.at(limb);
    limbLengthInequalityConstraintsMaxValues_(i) = maxLimbLenghts.at(limb);
    ++i;
  }
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

  // Update adapter.
  const auto& poseParameterization = dynamic_cast<const PoseParameterization&>(p);
  const Pose basePose = poseParameterization.getPose();

  // Support region.
  // TODO Remove adapter!
  values.segment(0, nSupportRegionInequalityConstraints_) =
      supportRegionInequalityConstraintGlobalJacobian_ * basePose.getPosition().vector().head(2);
//      supportRegionInequalityConstraintGlobalJacobian_ * adapter_.getCenterOfMassInWorldFrame().vector().head(2);

  // Leg length.
  size_t i(0);
  for (const auto& limb : adapter_.getLimbs()) {
    const Position footPosition(stance_.at(limb));
    const Position baseToFootInBase = basePose.getRotation().inverseRotate(footPosition - basePose.getPosition());
    const auto baseToHipInBase = adapter_.getPositionBaseToHipInBaseFrame(limb);
    const double legLength = Vector(baseToFootInBase - baseToHipInBase).norm();
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

void PoseOptimizationFunctionConstraints::updateNumberOfInequalityConstraints()
{
  setNumberOfInequalityConstraints(
      nSupportRegionInequalityConstraints_ + nLimbLengthInequalityConstraints_);
}

} /* namespace */
