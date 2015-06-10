/*
 * PoseOptimization.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/PoseOptimization.hpp>

namespace free_gait {

PoseOptimization::PoseOptimization()
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
  return true;
}

} /* namespace */
