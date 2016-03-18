/*
 * AdapterBase.cpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/executor/AdapterBase.hpp>

namespace free_gait {

AdapterBase::AdapterBase()
{
}

AdapterBase::~AdapterBase()
{
}

bool free_gait::AdapterBase::frameIdExists(const std::string& frameId)
{
  if (frameId == "base") return true;
  if (frameId == "odom") return true;
  if (frameId == "map") return true;
  return false;
}

Position free_gait::AdapterBase::transformPosition(const std::string& inputFrameId,
                                                   const std::string& outputFrameId,
                                                   const Position& position)
{
  Position transformedPosition;
  bool frameError = false;

  if (inputFrameId == "base") {

    if (outputFrameId == "base") {
      transformedPosition = position;
    } else if (outputFrameId == "odom") {
      transformedPosition = getPositionWorldToBaseInWorldFrame() + getOrientationWorldToBase().inverseRotate(position);
    } else {
      frameError = true;
    }

  } else if (inputFrameId == "odom") {

    if (outputFrameId == "base") {
      transformedPosition = getOrientationWorldToBase().rotate(position - getPositionWorldToBaseInWorldFrame());
    } else if (outputFrameId == "odom") {
      transformedPosition = position;
    } else {
      frameError = true;
    }

  } else {
    frameError = true;
  }

  if (frameError) throw std::invalid_argument("Invalid frame for transforming position.");
  return transformedPosition;
}

} /* namespace free_gait */
