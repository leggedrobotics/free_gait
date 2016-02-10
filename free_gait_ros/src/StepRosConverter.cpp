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
    const auto& limb = executor_->getAdapter().getLimbEnumFromLimbString(footstepMessage.name);
    Footstep footstep(limb);
    if (!fromMessage(footstepMessage, footstep)) return false;
    step.addLegMotion(limb, footstep);
  }

  for (const auto& legModeMessage : message.leg_mode) {
      const auto& limb = executor_->getAdapter().getLimbEnumFromLimbString(legModeMessage.name);
      LegMode legMode(limb);
      if (!fromMessage(legModeMessage, legMode)) return false;
      step.addLegMotion(limb, legMode);
    }

  for (const auto& jointTrajectoryMessage : message.joint_trajectory) {
    const auto& limb = executor_->getAdapter().getLimbEnumFromLimbString(jointTrajectoryMessage.name);
    JointTrajectory jointTrajectory(limb);
    if (!fromMessage(jointTrajectoryMessage, jointTrajectory)) return false;
    step.addLegMotion(limb, jointTrajectory);
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
  // Limb.
  foostep.limb_ = executor_->getAdapter().getLimbEnumFromLimbString(message.name);

  // Target.
  foostep.frameId_ = message.target.header.frame_id;
  Position target;
  kindr::phys_quant::eigen_impl::convertFromRosGeometryMsg(message.target.point, target);
  foostep.target_ = target;

  // Profile.
  foostep.profileHeight_ = message.profile_height;
  foostep.profileType_ = message.profile_type;

  // Average Velocity.
  foostep.averageVelocity_ = message.average_velocity;

  // Surface normal.
  Vector surfaceNormal;
  kindr::phys_quant::eigen_impl::convertFromRosGeometryMsg(message.surface_normal.vector, surfaceNormal);
  foostep.surfaceNormal_.reset(new Vector(surfaceNormal));

  // Ignore contact.
  foostep.ignoreContact_ = message.ignore_contact;

  // Ignore for pose adaptation.
  foostep.ignoreForPoseAdaptation_ = message.ignore_for_pose_adaptation;

  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::LegMode& message, LegMode& legMode)
{
  // Limb.
  legMode.limb_ = executor_->getAdapter().getLimbEnumFromLimbString(message.name);

  // Frame id. // TODO
//  foostep.frameId_ = message.target.header.frame_id;

  // Support mode.
  legMode.ignoreContact_ = !message.support_leg;

  // Duration.
  legMode.duration_ = ros::Duration(message.duration).toSec();

  // Surface normal.
  Vector surfaceNormal;
  kindr::phys_quant::eigen_impl::convertFromRosGeometryMsg(message.surface_normal.vector, surfaceNormal);
  legMode.surfaceNormal_.reset(new Vector(surfaceNormal));

  // Ignore for pose adaptation.
  legMode.ignoreForPoseAdaptation_ = message.ignore_for_pose_adaptation;

  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::JointTrajectory& message, JointTrajectory& jointTrajectory)
{
  // Limb.
  jointTrajectory.limb_ = executor_->getAdapter().getLimbEnumFromLimbString(message.name);

  // Trajectory.
  jointTrajectory.controlSetup_[ControlLevel::Position] = false;
  jointTrajectory.controlSetup_[ControlLevel::Velocity] = false;
  jointTrajectory.controlSetup_[ControlLevel::Acceleration] = false;
  jointTrajectory.controlSetup_[ControlLevel::Effort] = false;

  for (const auto& point : message.trajectory.points) {
    if (!point.positions.empty()) jointTrajectory.controlSetup_[ControlLevel::Position] = true;
    if (!point.velocities.empty()) jointTrajectory.controlSetup_[ControlLevel::Velocity] = true;
    if (!point.accelerations.empty()) jointTrajectory.controlSetup_[ControlLevel::Acceleration] = true;
    if (!point.effort.empty()) jointTrajectory.controlSetup_[ControlLevel::Effort] = true;
  }

  for (const auto& controlSetup : jointTrajectory.controlSetup_) {
    if (!controlSetup.second) continue;
    jointTrajectory.times_[controlSetup.first] = std::vector<JointTrajectory::Time>();
    jointTrajectory.values_[controlSetup.first] = std::vector<std::vector<JointTrajectory::ValueType>>();
    jointTrajectory.trajectories_[controlSetup.first] = std::vector<curves::PolynomialSplineQuinticScalarCurve>();
  }

  for (const auto& point : message.trajectory.points) {
    if (!point.positions.empty()) jointTrajectory.times_[ControlLevel::Position].push_back(ros::Duration(point.time_from_start).toSec());
    if (!point.velocities.empty()) jointTrajectory.times_[ControlLevel::Velocity].push_back(ros::Duration(point.time_from_start).toSec());
    if (!point.accelerations.empty()) jointTrajectory.times_[ControlLevel::Acceleration].push_back(ros::Duration(point.time_from_start).toSec());
    if (!point.effort.empty()) jointTrajectory.times_[ControlLevel::Effort].push_back(ros::Duration(point.time_from_start).toSec());
  }

  size_t nJoints = message.trajectory.joint_names.size();
  for (const auto& controlSetup : jointTrajectory.controlSetup_) {
    for (size_t j = 0; j < nJoints; ++j) {
      if (!controlSetup.second) continue;
      jointTrajectory.values_[controlSetup.first].push_back(std::vector<JointTrajectory::ValueType>());
      for (const auto& point : message.trajectory.points) {
        if (controlSetup.first == ControlLevel::Position && !point.positions.empty()) {
          jointTrajectory.values_[controlSetup.first][j].push_back(point.positions[j]);
        } else if (controlSetup.first == ControlLevel::Velocity && !point.velocities.empty()) {
          jointTrajectory.values_[controlSetup.first][j].push_back(point.velocities[j]);
        } else if (controlSetup.first == ControlLevel::Acceleration && !point.accelerations.empty()) {
          jointTrajectory.values_[controlSetup.first][j].push_back(point.accelerations[j]);
        } else if (controlSetup.first == ControlLevel::Effort && !point.effort.empty()) {
          jointTrajectory.values_[controlSetup.first][j].push_back(point.effort[j]);
        }
      }
    }
  }

  // Surface normal.
  Vector surfaceNormal;
  kindr::phys_quant::eigen_impl::convertFromRosGeometryMsg(message.surface_normal.vector, surfaceNormal);
  jointTrajectory.surfaceNormal_.reset(new Vector(surfaceNormal));

  // Ignore contact.
  jointTrajectory.ignoreContact_ = message.ignore_contact;

  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::BaseAuto& message,
                                   BaseAuto& baseAuto)
{
  baseAuto.height_.reset(new double(message.height));
  baseAuto.averageLinearVelocity_ = message.average_linear_velocity;
  baseAuto.averageAngularVelocity_ = message.average_angular_velocity;
  baseAuto.supportMargin_ = message.support_margin;
  return true;
}

} /* namespace */
