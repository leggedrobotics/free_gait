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

StepRosConverter::StepRosConverter(std::shared_ptr<quadruped_model::QuadrupedModel> quadrupedModel)
    : quadrupedModel_(quadrupedModel)
{

}

StepRosConverter::~StepRosConverter()
{

}

Step::State StepRosConverter::getStepStateFromString(const std::string& stateName)
{
  Step::State state;
  if (stateName == "pre_step")
    state = Step::State::PreStep;
  else if (stateName == "at_step")
    state = Step::State::AtStep;
  else if (stateName == "post_step")
    state = Step::State::PostStep;
  else {
    ROS_ERROR_STREAM("Invalid step state name: " << stateName << ".");
    return Step::State::Undefined;
  }
}


bool StepRosConverter::fromMessage(const free_gait_msgs::Step& message, free_gait::Step& step)
{

  // Leg motion.
  for (const auto& footstepMessage : message.footstep) {
    Footstep footstep;
    if (!fromMessage(footstepMessage, footstep)) return false;
    const auto& limb = quadrupedModel_->getLimbEnumFromLimbString(footstepMessage.name);
    step.addLegMotion(limb, footstep);
  }

  // Base motion.
  for (const auto& baseAutoMessage : message.base_auto) {
    BaseAuto baseAuto;
    if (!fromMessage(baseAutoMessage, baseAuto)) return false;
    const auto& state = getStepStateFromString(baseAutoMessage.state);
    step.addBaseMotion(state, baseAuto);
  }

  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::Footstep& message,
                                   Footstep& foostep)
{
  // Target.
  foostep.setFrameId(message.target.header.frame_id);
  Position target;
  kindr::phys_quant::eigen_impl::convertFromRosGeometryMsg(message.target.point, target);
  foostep.setTarget(target);

  // Profile.
  foostep.setProfileHeight(message.profile_height);
  foostep.setProfileType(message.profile_type);

  // Average Velocity.
  foostep.setAverageVelocity(message.average_velocity);

  // Surface normal.
  Vector surfaceNormal;
  kindr::phys_quant::eigen_impl::convertFromRosGeometryMsg(message.surface_normal.vector, surfaceNormal);
  foostep.setSurfaceNormal(surfaceNormal);

  // Ignore contact.
  foostep.setIgnoreContact(message.ignore_contact);

  // Ignore for pose adaptation.
  foostep.setIgnoreForPoseAdaptation(message.ignore_for_pose_adaptation);

  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::BaseAuto& message,
                                   BaseAuto& baseAuto)
{
  baseAuto.setIgnore(message.ignore);
  baseAuto.setHeight(message.height);
  baseAuto.setAverageVelocity(message.average_velocity);
  return true;
}

} /* namespace */
