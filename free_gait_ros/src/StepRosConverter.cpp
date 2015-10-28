/*
 * StepRosConverter.cpp
 *
 *  Created on: Feb 25, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_ros/StepRosConverter.hpp"

// Quadruped model
#include "quadruped_model/QuadrupedModel.hpp"

// Kindr
#include "kindr/thirdparty/ros/RosGeometryMsgPhysicalQuantitiesEigen.hpp"

namespace free_gait {

StepRosConverter::StepRosConverter(std::shared_ptr<Executor> executor)
    : executor_(executor)
{

}

StepRosConverter::~StepRosConverter()
{

}

bool StepRosConverter::fromMessage(const free_gait_msgs::Step& message, free_gait::Step& step)
{

  // Leg motion.
  for (const auto& footstepMessage : message.footstep) {
    Footstep footstep;
    if (!fromMessage(footstepMessage, footstep)) return false;
    const auto& limb = executor_->getAdapter().getLimbEnumFromLimbString(footstepMessage.name);
    step.addLegMotion(limb, footstep);
  }

  // Base motion.
  for (const auto& baseAutoMessage : message.base_auto) {
    BaseAuto baseAuto;
    if (!fromMessage(baseAutoMessage, baseAuto)) return false;
    step.addBaseMotion(baseAuto);
  }

  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::Footstep& message,
                                   Footstep& foostep)
{
  // Target.
  foostep.frameId_ = message.target.header.frame_id;
  Position target;
  kindr::phys_quant::eigen_impl::convertFromRosGeometryMsg(message.target.point, target);
  foostep.target_ = target;

  // Profile.
  foostep.profileHeight_ = message.profile_height;
  foostep.profileType_ =message.profile_type;

  // Average Velocity.
  foostep.averageVelocity_ = message.average_velocity;

  // Surface normal.
  Vector surfaceNormal;
  kindr::phys_quant::eigen_impl::convertFromRosGeometryMsg(message.surface_normal.vector, surfaceNormal);
  foostep.surfaceNormal_ = surfaceNormal;

  // Ignore contact.
  foostep.ignoreContact_ = message.ignore_contact;

  // Ignore for pose adaptation.
  foostep.ignoreForPoseAdaptation_ = message.ignore_for_pose_adaptation;

  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::BaseAuto& message,
                                   BaseAuto& baseAuto)
{
  baseAuto.height_ = message.height;
  baseAuto.averageLinearVelocity_ = message.average_linear_velocity;
  baseAuto.averageAngularVelocity_ = message.average_angular_velocity;
  baseAuto.supportMargin_ = message.support_margin;
  return true;
}

} /* namespace */
