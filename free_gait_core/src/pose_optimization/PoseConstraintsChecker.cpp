/*
 * PoseConstraintsChecker.cpp
 *
 *  Created on: Aug 31, 2017
 *      Author: Péter Fankhauser
 */

#include "free_gait_core/pose_optimization/PoseConstraintsChecker.hpp"

namespace free_gait {

PoseConstraintsChecker::PoseConstraintsChecker(const AdapterBase& adapter)
    : PoseOptimizationBase(adapter),
      centerOfMassTolerance_(0.0),
      legLengthTolerance_(0.0)
{
}

PoseConstraintsChecker::~PoseConstraintsChecker()
{
}

void PoseConstraintsChecker::setTolerances(const double centerOfMassTolerance, const double legLengthTolerance)
{
  centerOfMassTolerance_ = centerOfMassTolerance;
  legLengthTolerance_ = legLengthTolerance;
}

bool PoseConstraintsChecker::check(const Pose& pose)
{
  state_.setPoseBaseToWorld(pose);
  adapter_.setInternalDataFromState(state_); // To guide IK.
  if (!updateJointPositionsInState(state_)) {
    return false;
  }
  adapter_.setInternalDataFromState(state_);

  // Check center of mass.
  grid_map::Polygon supportRegionCopy(supportRegion_);
  supportRegionCopy.offsetInward(centerOfMassTolerance_);
  if (!supportRegion_.isInside(adapter_.getCenterOfMassInWorldFrame().vector().head(2))) {
    return false;
  }

  // Check leg length. TODO Replace with joint limits?
  for (const auto& foot : stance_) {
    const Position footPositionInBase(
        adapter_.transformPosition(adapter_.getWorldFrameId(), adapter_.getBaseFrameId(), foot.second));
    const double legLength = Vector(footPositionInBase - adapter_.getPositionBaseToHipInBaseFrame(foot.first)).norm();
    if (legLength < minLimbLenghts_[foot.first] - legLengthTolerance_ || legLength > maxLimbLenghts_[foot.first] + legLengthTolerance_) {
      return false;
    }
  }

  return true;
}

}
