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

bool AdapterBase::frameIdExists(const std::string& frameId) const
{
  if (frameId == "base") return true;
  if (frameId == "odom") return true;
  if (frameId == "map") return true;
  if (frameId == "map_ga") return true;
  return false;
}

Position AdapterBase::transformPosition(const std::string& inputFrameId,
                                                   const std::string& outputFrameId,
                                                   const Position& position) const
{
  Position transformedPosition;
  bool frameError = false;

  if (inputFrameId == "base") {

    if (outputFrameId == "base") {
      transformedPosition = position;
    } else if (outputFrameId == "odom") {
      transformedPosition = getPositionWorldToBaseInWorldFrame() + getOrientationWorldToBase().inverseRotate(position);
    } else if (outputFrameId == "map" || outputFrameId == "map_ga" ) {
      const Position positionInOdom = transformPosition(inputFrameId, "odom", position);
      transformedPosition = transformPosition("odom", outputFrameId, positionInOdom);
    } else {
      frameError = true;
    }

  } else if (inputFrameId == "odom") {

    if (outputFrameId == "base") {
      transformedPosition = getOrientationWorldToBase().rotate(position - getPositionWorldToBaseInWorldFrame());
    } else if (outputFrameId == "odom") {
      transformedPosition = position;
    } else if (outputFrameId == "map" || outputFrameId == "map_ga" ) {
      // TODO Why does this not work?
//      transformedPosition = getFramePoseInWorld(outputFrameId).inverseTransform(position);
      const Pose pose(getFramePoseInWorld(outputFrameId));
      transformedPosition = pose.getRotation().rotate((position - pose.getPosition()));
    } else {
      frameError = true;
    }

  } else if (inputFrameId == "map" || inputFrameId == "map_ga") {

    if (outputFrameId == "base") {
      const Position positionInOdom = transformPosition(inputFrameId, "odom", position);
      transformedPosition = transformPosition("odom", outputFrameId, positionInOdom);
    } else if (outputFrameId == "odom") {
      // TODO Why does this not work?
//      transformedPosition = getFramePoseInWorld(inputFrameId).transform(position);
      const Pose pose(getFramePoseInWorld(inputFrameId));
      transformedPosition = pose.getRotation().inverseRotate(position) + pose.getPosition();
    } else {
      frameError = true;
    }

  } else {
    frameError = true;
  }

  if (frameError) {
    const std::string message = "Invalid frame for transforming position (input frame: " + inputFrameId + ", output frame: " + outputFrameId + ").";
    throw std::invalid_argument(message);
  }
  return transformedPosition;
}

} /* namespace free_gait */
