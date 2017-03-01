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
                                          const Transform& transformInSourceFrame,
                                          const ros::Time& time)
{
  for (Step& step : stepQueue.queue_) {
    if (!adaptCoordinates(step, sourceFrameId, targetFrameId, transformInSourceFrame, time)) return false;
  }
  return true;
}


bool StepFrameConverter::adaptCoordinates(Step& step, const std::string& sourceFrameId,
                                          const std::string& targetFrameId,
                                          const Transform& transformInSourceFrame,
                                          const ros::Time& time)
{
  // Leg motions.
  for (const auto& legMotion : step.getLegMotions()) {

    // Foostep.
    if (legMotion.second->getType() == LegMotionBase::Type::Footstep) {
      Footstep& footstep = dynamic_cast<Footstep&>(*(legMotion.second));
      if (!adaptCoordinates(footstep, sourceFrameId, targetFrameId, transformInSourceFrame, time)) return false;
    }

    // EndEffectorTrajectory.
    if (legMotion.second->getType() == LegMotionBase::Type::EndEffectorTrajectory) {
      EndEffectorTrajectory& endEffectorTrajectory = dynamic_cast<EndEffectorTrajectory&>(*(legMotion.second));
      if (!adaptCoordinates(endEffectorTrajectory, sourceFrameId, targetFrameId, transformInSourceFrame, time)) return false;
    }

  }

  // Base motion.
  if (step.hasBaseMotion()) {

    const auto& baseMotion = step.getBaseMotion();

    // Base Auto.
//    if (baseMotion.getType() == BaseMotionBase::Type::Auto) {
//      BaseAuto& baseAuto = dynamic_cast<BaseAuto&>(baseMotion);
//      if (!adaptCoordinates(baseAuto, sourceFrameId, targetFrameId, transformInTargetFrame, time)) return false;
//    }

    // Base Trajectory.
    if (baseMotion.getType() == BaseMotionBase::Type::Trajectory){
      const BaseTrajectory& baseTrajectory = dynamic_cast<const BaseTrajectory&>(baseMotion);
      if (!adaptCoordinates(const_cast<BaseTrajectory&> (baseTrajectory), sourceFrameId, targetFrameId, transformInSourceFrame, time)) return false;
    }
  }
  return true;
}

bool StepFrameConverter::adaptCoordinates(Footstep& footstep, const std::string& sourceFrameId,
                                          const std::string& targetFrameId,
                                          const Transform& transformInSourceFrame,
                                          const ros::Time& time)
{
  Transform transform;
  if (footstep.getFrameId(ControlLevel::Position) == sourceFrameId) {
    if (!getTransform(sourceFrameId, targetFrameId, transformInSourceFrame, time, transform)) return false;
    footstep.target_ = transform.transform(footstep.target_);
    footstep.frameId_ = targetFrameId;
  }

  return true;
}

bool StepFrameConverter::adaptCoordinates(EndEffectorTrajectory& endEffectorTrajectory, const std::string& sourceFrameId,
                                          const std::string& targetFrameId,
                                          const Transform& transformInSourceFrame,
                                          const ros::Time& time)
{
  Transform transform;
  if (endEffectorTrajectory.getFrameId(ControlLevel::Position) == sourceFrameId) {
    if (!getTransform(sourceFrameId, targetFrameId, transformInSourceFrame, time, transform)) return false;
    for (auto& knot : endEffectorTrajectory.values_.at(ControlLevel::Position)){
      knot = (transform.transform(Position(knot))).vector();
    }
    endEffectorTrajectory.frameIds_.at(ControlLevel::Position) = targetFrameId;
  }

  return true;
}

bool StepFrameConverter::adaptCoordinates(BaseTrajectory& baseTrajectory, const std::string& sourceFrameId,
                                          const std::string& targetFrameId,
                                          const Transform& transformInSourceFrame,
                                          const ros::Time& time)
{
  Transform transform;
  if (baseTrajectory.getFrameId(ControlLevel::Position) == sourceFrameId) {
    if (!getTransform(sourceFrameId, targetFrameId, transformInSourceFrame, time, transform)) return false;
    for (auto& knot : baseTrajectory.values_.at(ControlLevel::Position)){
      knot.getPosition() = transform.transform(knot.getPosition());
      knot.getRotation() = transform.getRotation()*knot.getRotation();
    }
    baseTrajectory.frameIds_.at(ControlLevel::Position) = targetFrameId;
  }

  return true;
}

bool StepFrameConverter::getTransform(const std::string& sourceFrameId,
                                      const std::string& targetFrameId,
                                      const Transform& transformInSourceFrame,
                                      const ros::Time& time, Transform& transform)
{
  if (sourceFrameId == cachedSourceFrameId_ && targetFrameId == cachedTargetFrameId_
      && transformInSourceFrame.getPosition() == cachedTransformInSourceFrame_.getPosition()
      && transformInSourceFrame.getRotation() == cachedTransformInSourceFrame_.getRotation()
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
  transform = transform * transformInSourceFrame;

  // Cache transform.
  cachedSourceFrameId_ = sourceFrameId;
  cachedTargetFrameId_ = targetFrameId;
  cachedTransformInSourceFrame_ = transformInSourceFrame;
  cachedTime_ = time;
  cachedTransform_ = transform;

  return true;
}

} /* namespace free_gait */
