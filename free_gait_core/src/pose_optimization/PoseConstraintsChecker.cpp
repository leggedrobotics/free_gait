/*
 * PoseConstraintsChecker.cpp
 *
 *  Created on: Aug 31, 2017
 *      Author: Péter Fankhauser
 */

#include "free_gait_core/pose_optimization/PoseConstraintsChecker.hpp"

namespace free_gait {

PoseConstraintsChecker::PoseConstraintsChecker(const AdapterBase& adapter)
    : PoseOptimizationBase(adapter)
{
}

PoseConstraintsChecker::~PoseConstraintsChecker()
{
}

bool PoseConstraintsChecker::check(const Pose& pose)
{
  state_.setPoseBaseToWorld(pose);
  adapter_.setInternalDataFromState(state_); // To guide IK.
  if (!updateJointPositionsInState(state_)) return false;
  adapter_.setInternalDataFromState(state_);

  // Check center of mass.
  if (!supportRegion_.isInside(adapter_.getCenterOfMassInWorldFrame().vector().head(2))) return false;

  // Check leg length. TODO Replace with joint limits?
  size_t i(0);
  for (const auto& limb : adapter_.getLimbs()) {
    const double legLength = Vector(adapter_.getPositionBaseToFootInBaseFrame(limb) - adapter_.getPositionBaseToHipInBaseFrame(limb)).norm();
    if (legLength < minLimbLenghts_[limb] || legLength > maxLimbLenghts_[limb]) return false;
  }

  return true;
}

}
