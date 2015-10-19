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
  for (const auto& footTargetMessage : message.foot_target) {
    FootTarget footTarget;
    if (!fromMessage(footTargetMessage, footTarget)) return false;
    const auto& limb = quadrupedModel_->getLimbEnumFromLimbString(footTargetMessage.name);
    step.addLegMotion(limb, footTarget);
  }

  // Base motion.
  for (const auto& baseAutoMessage : message.base_auto) {
    BaseAuto baseAuto;
//    if (!fromMessage(baseAutoMessage, baseAuto)) return false;
//    const auto& limb = quadrupedModel_->getLimbEnumFromLimbString(footTargetMessage.name);
//    step.addLegMotion(limb, footTarget);
  }


//
//  // Base shift data.
//  if (message.ignore_base_shift) {
//    BaseShiftData baseShiftData;
//    BaseShiftProfile trajectory; // Hack to make clone() work.
//    baseShiftData.setTrajectory(trajectory);
//    baseShiftData.setIgnore(true);
//    addBaseShiftData(Step::State::PreStep, baseShiftData);
//    addBaseShiftData(Step::State::AtStep, baseShiftData);
//    addBaseShiftData(Step::State::PostStep, baseShiftData);
//  } else {
//    for (const auto& baseShiftMessage : message.base_shift_data) {
//      BaseShiftDataRosWrapper baseShiftData;
//      if (!baseShiftData.fromMessage(baseShiftMessage)) return false;
//
//      // State type.
//      Step::State state;
//      if (baseShiftMessage.name == "pre_step")
//        state = Step::State::PreStep;
//      else if (baseShiftMessage.name == "at_step")
//        state = Step::State::AtStep;
//      else if (baseShiftMessage.name == "post_step")
//        state = Step::State::PostStep;
//      else {
//        ROS_ERROR_STREAM("Invalid base shift state name: " << baseShiftMessage.name << ".");
//        return false;
//      }
//      addBaseShiftData(state, baseShiftData);
//    }
//  }

  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::FootTarget& message,
                                   FootTarget& footTarget)
{
  // Target.
  footTarget.setFrameId(message.target.header.frame_id);
  Position target;
  kindr::phys_quant::eigen_impl::convertFromRosGeometryMsg(message.target.point, target);
  footTarget.setTarget(target);

  // Profile.
  footTarget.setProfileHeight(message.profile_height);
  footTarget.setProfileType(message.profile_type);

  // Average Velocity.
  footTarget.setAverageVelocity(message.average_velocity);

  // Surface normal.
  Vector surfaceNormal;
  kindr::phys_quant::eigen_impl::convertFromRosGeometryMsg(message.surface_normal.vector, surfaceNormal);
  footTarget.setSurfaceNormal(surfaceNormal);

  // No touchdown.
  footTarget.setNoTouchdown(message.no_touchdown);

  // Ignore for pose adaptation.
  footTarget.setIgnoreForPoseAdaptation(message.ignore_for_pose_adaptation);

  return true;
}

} /* namespace */
