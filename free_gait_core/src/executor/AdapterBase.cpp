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
  if (frameId == getWorldFrameId()) return true;
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
    } else if (outputFrameId == getWorldFrameId()) {
      transformedPosition = getPositionWorldToBaseInWorldFrame() + getOrientationWorldToBase().inverseRotate(position);
    } else if (outputFrameId == "map" || outputFrameId == "map_ga" ) {
      const Position positionInOdom = transformPosition(inputFrameId, getWorldFrameId(), position);
      transformedPosition = transformPosition(getWorldFrameId(), outputFrameId, positionInOdom);
    } else {
      frameError = true;
    }

  } else if (inputFrameId == getWorldFrameId()) {

    if (outputFrameId == "base") {
      transformedPosition = getOrientationWorldToBase().rotate(position - getPositionWorldToBaseInWorldFrame());
    } else if (outputFrameId == getWorldFrameId()) {
      transformedPosition = position;
    } else if (outputFrameId == "map" || outputFrameId == "map_ga" ) {
      // TODO Why does this not work?
//      transformedPosition = getFramePoseInWorld(outputFrameId).inverseTransform(position);
      const Pose pose(getFrameTransform(outputFrameId));
      transformedPosition = pose.getRotation().rotate((position - pose.getPosition()));
    } else {
      frameError = true;
    }

  } else if (inputFrameId == "map" || inputFrameId == "map_ga") {

    if (outputFrameId == "base") {
      const Position positionInOdom = transformPosition(inputFrameId, getWorldFrameId(), position);
      transformedPosition = transformPosition(getWorldFrameId(), outputFrameId, positionInOdom);
    } else if (outputFrameId == getWorldFrameId()) {
      // TODO Why does this not work?
//      transformedPosition = getFramePoseInWorld(inputFrameId).transform(position);
      const Pose pose(getFrameTransform(inputFrameId));
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

RotationQuaternion AdapterBase::transformOrientation(const std::string& inputFrameId,
                                                     const std::string& outputFrameId,
                                                     const RotationQuaternion& orientation) const
{
  RotationQuaternion transformedOrientation;
  bool frameError = false;

  if (inputFrameId == getWorldFrameId()) {

    if (outputFrameId == getWorldFrameId()) {
      transformedOrientation = orientation;
    } else if (outputFrameId == "map" || outputFrameId == "map_ga" ) {
      const Pose pose(getFrameTransform(outputFrameId));
      transformedOrientation = pose.getRotation().inverted() * orientation;
    } else {
      frameError = true;
    }

  } else if (inputFrameId == "map" || inputFrameId == "map_ga") {

    if (outputFrameId == getWorldFrameId()) {
      const Pose pose(getFrameTransform(inputFrameId));
      transformedOrientation = pose.getRotation() * orientation;
    } else {
      frameError = true;
    }

  } else {
    frameError = true;
  }

  if (frameError) {
    const std::string message = "Invalid frame for transforming orientation (input frame: " + inputFrameId + ", output frame: " + outputFrameId + ").";
    throw std::invalid_argument(message);
  }
  return transformedOrientation;
}

Pose AdapterBase::transformPose(const std::string& inputFrameId, const std::string& outputFrameId,
                                const Pose& pose) const
{
  Pose transformedPose;
  transformedPose.getPosition() = transformPosition(inputFrameId, outputFrameId, pose.getPosition());
  transformedPose.getRotation() = transformOrientation(inputFrameId, outputFrameId, pose.getRotation());
  return transformedPose;
}

} /* namespace free_gait */
