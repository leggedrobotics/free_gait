/*
 * StepFrameConverter.hpp
 *
 *  Created on: Nov 11, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include <free_gait_core/free_gait_core.hpp>

// ROS
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

// STD
#include <memory>
#include <string>

namespace free_gait {

class StepFrameConverter
{
 public:
  StepFrameConverter(tf2_ros::Buffer& tfBuffer);
  virtual ~StepFrameConverter();

  bool adaptCoordinates(StepQueue& stepQueue, const std::string& sourceFrameId,
                        const std::string& targetFrameId,
                        const Transform& transformInSourceFrame = Transform(),
                        const ros::Time& time = ros::Time(0));

  bool adaptCoordinates(Step& step, const std::string& sourceFrameId,
                        const std::string& targetFrameId,
                        const Transform& transformInSourceFrame = Transform(),
                        const ros::Time& time = ros::Time(0));

  bool adaptCoordinates(Footstep& footstep, const std::string& sourceFrameId,
                        const std::string& targetFrameId,
                        const Transform& transformInSourceFrame = Transform(),
                        const ros::Time& time = ros::Time(0));

  bool adaptCoordinates(EndEffectorTrajectory& endEffectorTrajectory, const std::string& sourceFrameId,
                        const std::string& targetFrameId,
                        const Transform& transformInSourceFrame = Transform(),
                        const ros::Time& time = ros::Time(0));

  bool adaptCoordinates(BaseTrajectory& baseTrajectory, const std::string& sourceFrameId,
                        const std::string& targetFrameId,
                        const Transform& transformInSourceFrame,
                        const ros::Time& time);

 private:

  bool getTransform(const std::string& sourceFrameId, const std::string& targetFrameId,
                    const Transform& transformInSourceFrame, const ros::Time& time,
                    Transform& transform);

  /// TF buffer used to read the transformations.
  /// Note: Needs to be updated from outside with
  /// a TF Listener!
  tf2_ros::Buffer& tfBuffer_;

  /// Cached transform for faster conversion.
  std::string cachedTargetFrameId_, cachedSourceFrameId_;
  ros::Time cachedTime_;
  Transform cachedTransformInSourceFrame_;
  Transform cachedTransform_;
};

} /* namespace free_gait */
