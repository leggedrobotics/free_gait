/*
 * StepFrameConverter.cpp
 *
 *  Created on: Nov 11, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <free_gait_ros/StepFrameConverter.hpp>
#include <kindr_ros/kindr_ros.hpp>

namespace free_gait {

StepFrameConverter::StepFrameConverter(std::shared_ptr<tf2_ros::Buffer> tfBuffer)
    : tfBuffer_(tfBuffer)
{
}

StepFrameConverter::~StepFrameConverter()
{
}

bool StepFrameConverter::adaptCoordinates(StepQueue& stepQueue, const std::string& sourceFrameId,
                                          const std::string& targetFrameId,
                                          const Transform& transformInTargetFrame,
                                          const ros::Time& time)
{
  for (Step& step : stepQueue.queue_) {
    if (!adaptCoordinates(step, sourceFrameId, targetFrameId, transformInTargetFrame, time)) return false;
  }
  return true;
}


bool StepFrameConverter::adaptCoordinates(Step& step, const std::string& sourceFrameId,
                                          const std::string& targetFrameId,
                                          const Transform& transformInTargetFrame,
                                          const ros::Time& time)
{
  // Leg motions.
  for (const auto& legMotion : step.getLegMotions()) {

    // Foostep.
    if (legMotion.second->getType() == LegMotionBase::Type::Footstep) {
      Footstep& footstep = dynamic_cast<Footstep&>(*(legMotion.second));
      if (!adaptCoordinates(footstep, sourceFrameId, targetFrameId, transformInTargetFrame, time)) return false;
    }

  }

//  // Base motion.
//  if (step.hasBaseMotion()) {
//
//    const auto& baseMotion = step.getBaseMotion();
//
//    // Base Auto.
//    if (baseMotion.getType() == BaseMotionBase::Type::Auto) {
//      BaseAuto& baseAuto = dynamic_cast<BaseAuto&>(baseMotion);
//      if (!adaptCoordinates(baseAuto, sourceFrameId, targetFrameId, transformInTargetFrame, time)) return false;
//    }
//  }
  return true;
}

bool StepFrameConverter::adaptCoordinates(Footstep& footstep, const std::string& sourceFrameId,
                                          const std::string& targetFrameId,
                                          const Transform& transformInTargetFrame,
                                          const ros::Time& time)
{
  Transform transform;
  if (footstep.getFrameId(ControlLevel::Position) == sourceFrameId) {
    if (!getTransform(sourceFrameId, targetFrameId, transformInTargetFrame, time, transform)) return false;
    footstep.target_ = transform.transform(footstep.target_);
    footstep.frameId_ = targetFrameId;
  }

  return true;
}

bool StepFrameConverter::getTransform(const std::string& sourceFrameId,
                                      const std::string& targetFrameId,
                                      const Transform& transformInTargetFrame,
                                      const ros::Time& time, Transform& transform)
{
  if (sourceFrameId == cachedSourceFrameId_ && targetFrameId == cachedTargetFrameId_
      && transformInTargetFrame.getPosition() == cachedTransformInTargetFrame_.getPosition()
      && transformInTargetFrame.getRotation() == cachedTransformInTargetFrame_.getRotation()
      && time == cachedTime_) {
    // No lookup required if already cached.
    transform = cachedTransform_;
    return true;
  }

  // Adding this leads to blocking `lookupTransform`. Why?
  std::string errorMessage;
  tfBuffer_->canTransform(targetFrameId, sourceFrameId, time, ros::Duration(5.0), &errorMessage);
  geometry_msgs::TransformStamped transformStamped;
  try {
    // TODO Why is `lookupTransform` not blocking here?
    transformStamped = tfBuffer_->lookupTransform(targetFrameId, sourceFrameId, time, ros::Duration(5.0));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  kindr_ros::convertFromRosGeometryMsg(transformStamped.transform, transform);
  transform = transform * transformInTargetFrame;

  // Cache transform.
  cachedSourceFrameId_ = sourceFrameId;
  cachedTargetFrameId_ = targetFrameId;
  cachedTransformInTargetFrame_ = transformInTargetFrame;
  cachedTime_ = time;
  cachedTransform_ = transform;

  return true;
}

} /* namespace free_gait */
