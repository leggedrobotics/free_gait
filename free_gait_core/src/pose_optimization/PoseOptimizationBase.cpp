/*
 * PoseOptimizationBase.cpp
 *
 *  Created on: Aug 31, 2017
 *      Author: Péter Fankhauser
 */

#include "free_gait_core/pose_optimization/PoseOptimizationBase.hpp"

namespace free_gait {

PoseOptimizationBase::PoseOptimizationBase(const AdapterBase& adapter)
    : adapter_(adapter)
{
}

PoseOptimizationBase::~PoseOptimizationBase()
{
}

void PoseOptimizationBase::setCurrentState(const State& state)
{
  state_ = state;
}

void PoseOptimizationBase::setStance(const Stance& stance)
{
  stance_ = stance;
}

void PoseOptimizationBase::setSupportStance(const Stance& supportStance)
{
  supportStance_ = supportStance;
}

void PoseOptimizationBase::setNominalStance(const Stance& nominalStanceInBaseFrame)
{
  nominalStanceInBaseFrame_ = nominalStanceInBaseFrame;
}

void PoseOptimizationBase::setSupportRegion(const grid_map::Polygon& supportRegion)
{
  supportRegion_ = supportRegion;
}

void PoseOptimizationBase::setLimbLengthConstraints(const LimbLengths& minLimbLenghts, const LimbLengths& maxLimbLenghts)
{
  minLimbLenghts_ = minLimbLenghts;
  maxLimbLenghts_ = maxLimbLenghts;
}

void PoseOptimizationBase::checkSupportRegion()
{
  if (supportRegion_.nVertices() == 0) {
    for (const auto& foot : supportStance_)
      supportRegion_.addVertex(foot.second.vector().head<2>());
  }
}

bool PoseOptimizationBase::updateJointPositionsInState(State& state) const
{
  bool totalSuccess = true;
  for (const auto& foot : stance_) {
    const Position footPositionInBase(
        adapter_.transformPosition(adapter_.getWorldFrameId(), adapter_.getBaseFrameId(), foot.second));
    JointPositionsLeg jointPositions;
    const bool success = adapter_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(footPositionInBase, foot.first, jointPositions);
    if (success) state.setJointPositionsForLimb(foot.first, jointPositions);
    else totalSuccess = totalSuccess && success;
  }
  return totalSuccess;
}

}
