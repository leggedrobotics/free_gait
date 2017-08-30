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
#include "kindr_ros/kindr_ros.hpp"

namespace free_gait {

StepRosConverter::StepRosConverter(const AdapterBase& adapter)
    : adapter_(adapter)
{
}

StepRosConverter::~StepRosConverter()
{
}

bool StepRosConverter::fromMessage(const std::vector<free_gait_msgs::Step>& message, std::vector<free_gait::Step>& steps)
{
  steps.resize(message.size());
  for (const auto& stepMessage : message) {
    Step step;
    fromMessage(stepMessage, step);
    steps.push_back(step);
  }
  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::Step& message, Step& step)
{
  // ID.
  step.setId(message.id);

  // Leg motion.
  for (const auto& footstepMessage : message.footstep) {
    const auto limb = adapter_.getLimbEnumFromLimbString(footstepMessage.name);
    Footstep footstep(limb);
    if (!fromMessage(footstepMessage, footstep)) return false;
    step.addLegMotion(footstep);
  }

  for (const auto& endEffectorTargetMessage : message.end_effector_target) {
    const auto limb = adapter_.getLimbEnumFromLimbString(endEffectorTargetMessage.name);
    EndEffectorTarget endEffectorTarget(limb);
    if (!fromMessage(endEffectorTargetMessage, endEffectorTarget)) return false;
    step.addLegMotion(endEffectorTarget);
  }

  for (const auto& endEffectorTrajectoryMessage : message.end_effector_trajectory) {
    const auto limb = adapter_.getLimbEnumFromLimbString(endEffectorTrajectoryMessage.name);
    EndEffectorTrajectory endEffectorTrajectory(limb);
    if (!fromMessage(endEffectorTrajectoryMessage, endEffectorTrajectory)) return false;
    step.addLegMotion(endEffectorTrajectory);
  }

  for (const auto& legModeMessage : message.leg_mode) {
      const auto limb = adapter_.getLimbEnumFromLimbString(legModeMessage.name);
      LegMode legMode(limb);
      if (!fromMessage(legModeMessage, legMode)) return false;
      step.addLegMotion(legMode);
    }

  for (const auto& jointTrajectoryMessage : message.joint_trajectory) {
    const auto limb = adapter_.getLimbEnumFromLimbString(jointTrajectoryMessage.name);
    JointTrajectory jointTrajectory(limb);
    if (!fromMessage(jointTrajectoryMessage, jointTrajectory)) return false;
    step.addLegMotion(jointTrajectory);
  }

  // Base motion.
  for (const auto& baseAutoMessage : message.base_auto) {
    BaseAuto baseAuto;
    if (!fromMessage(baseAutoMessage, baseAuto)) return false;
    step.addBaseMotion(baseAuto);
  }

  for (const auto& baseTargetMessage : message.base_target) {
    BaseTarget baseTarget;
    if (!fromMessage(baseTargetMessage, baseTarget)) return false;
    step.addBaseMotion(baseTarget);
  }

  for (const auto& baseTrajectoryMessage : message.base_trajectory) {
    BaseTrajectory baseTrajectory;
    if (!fromMessage(baseTrajectoryMessage, baseTrajectory)) return false;
    step.addBaseMotion(baseTrajectory);
  }

  // Custom command.
  for (const auto& customCommandMessage : message.custom_command) {
    CustomCommand customCommand;
    if (!fromMessage(customCommandMessage, customCommand)) return false;
    step.addCustomCommand(customCommand);
  }

  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::Footstep& message,
                                   Footstep& footstep)
{
  // Limb.
  footstep.limb_ = adapter_.getLimbEnumFromLimbString(message.name);

  // Target.
  footstep.frameId_ = message.target.header.frame_id;
  Position target;
  kindr_ros::convertFromRosGeometryMsg(message.target.point, target);
  footstep.target_ = target;

  // Profile.
  footstep.profileHeight_ = message.profile_height;
  footstep.profileType_ = message.profile_type;

  // Average Velocity.
  footstep.averageVelocity_ = message.average_velocity;

  // Surface normal.
  Vector surfaceNormal;
  kindr_ros::convertFromRosGeometryMsg(message.surface_normal.vector, surfaceNormal);
  footstep.surfaceNormal_.reset(new Vector(surfaceNormal));

  // Ignore contact.
  footstep.ignoreContact_ = message.ignore_contact;

  // Ignore for pose adaptation.
  footstep.ignoreForPoseAdaptation_ = message.ignore_for_pose_adaptation;

  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::EndEffectorTarget& message,
                                   EndEffectorTarget& endEffectorTarget)
{
  // Limb.
  endEffectorTarget.limb_ = adapter_.getLimbEnumFromLimbString(message.name);

  // Target position.
  endEffectorTarget.controlSetup_[ControlLevel::Position] = !message.target_position.empty();
  if (endEffectorTarget.controlSetup_[ControlLevel::Position]) {
    endEffectorTarget.frameIds_[ControlLevel::Position] = message.target_position[0].header.frame_id;
    Position targetPosition;
    kindr_ros::convertFromRosGeometryMsg(message.target_position[0].point, targetPosition);
    endEffectorTarget.targetPosition_ = targetPosition;
  }

  // Target velocity.
  endEffectorTarget.controlSetup_[ControlLevel::Velocity] = !message.target_velocity.empty();
  if (endEffectorTarget.controlSetup_[ControlLevel::Velocity]) {
    endEffectorTarget.frameIds_[ControlLevel::Velocity] = message.target_velocity[0].header.frame_id;
    LinearVelocity targetVelocity;
    kindr_ros::convertFromRosGeometryMsg(message.target_velocity[0].vector, targetVelocity);
    endEffectorTarget.targetVelocity_ = targetVelocity;
  }

  // TODO.
  endEffectorTarget.controlSetup_[ControlLevel::Acceleration] = !message.target_acceleration.empty();
  endEffectorTarget.controlSetup_[ControlLevel::Effort] = !message.target_force.empty();

  // Average Velocity.
  endEffectorTarget.averageVelocity_ = message.average_velocity;

  // Surface normal.
  Vector surfaceNormal;
  kindr_ros::convertFromRosGeometryMsg(message.surface_normal.vector, surfaceNormal);
  endEffectorTarget.surfaceNormal_.reset(new Vector(surfaceNormal));

  // Ignore contact.
  endEffectorTarget.ignoreContact_ = message.ignore_contact;

  // Ignore for pose adaptation.
  endEffectorTarget.ignoreForPoseAdaptation_ = message.ignore_for_pose_adaptation;

  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::EndEffectorTrajectory& message,
                                   EndEffectorTrajectory& endEffectorTrajectory)
{
  // Limb.
  endEffectorTrajectory.limb_ = adapter_.getLimbEnumFromLimbString(message.name);

  // Trajectory.
  endEffectorTrajectory.frameIds_[ControlLevel::Position] = message.trajectory.header.frame_id;

  for (const auto& point : message.trajectory.points) {
    if (!point.transforms.empty()) endEffectorTrajectory.controlSetup_[ControlLevel::Position] = true;
    if (!point.velocities.empty()) endEffectorTrajectory.controlSetup_[ControlLevel::Velocity] = true;
    if (!point.accelerations.empty()) endEffectorTrajectory.controlSetup_[ControlLevel::Acceleration] = true;
  }

  for (const auto& controlSetup : endEffectorTrajectory.controlSetup_) {
    if (!controlSetup.second) continue;
    endEffectorTrajectory.values_[controlSetup.first] = std::vector<EndEffectorTrajectory::ValueType>();
  }

  // TODO Copy times correctly for pure velocity or acceleration trajectories.
  for (const auto& point : message.trajectory.points) {
    if (!point.transforms.empty()) {
      endEffectorTrajectory.times_.push_back(ros::Duration(point.time_from_start).toSec());
    } else {
      std::cerr << "StepRosConverter: Could not read from ROS message, only position trajectories are supported for now." << std::endl;
      break;
    }
  }

  for (const auto& controlSetup : endEffectorTrajectory.controlSetup_) {
    if (!controlSetup.second)continue;
    for (const auto& point : message.trajectory.points) {
      if (controlSetup.first == ControlLevel::Position && !point.transforms.empty()) {
        Position position;
        kindr_ros::convertFromRosGeometryMsg(point.transforms[0].translation, position);
        endEffectorTrajectory.values_[controlSetup.first].push_back(position.vector());
      } else if (controlSetup.first == ControlLevel::Velocity && !point.velocities.empty()) {
//        baseTrajectory.derivatives_[controlSetup.first][j].push_back(point.velocities[j]);
      } else if (controlSetup.first == ControlLevel::Acceleration && !point.accelerations.empty()) {
//        baseTrajectory.derivatives_[controlSetup.first][j].push_back(point.accelerations[j]);
      } /*else if (controlSetup.first == ControlLevel::Effort && !point.effort.empty()) {
          baseTrajectory.derivatives_[controlSetup.first][j].push_back(point.effort[j]);
      }*/
    }
  }

  // Surface normal.
  Vector surfaceNormal;
  kindr_ros::convertFromRosGeometryMsg(message.surface_normal.vector, surfaceNormal);
  endEffectorTrajectory.surfaceNormal_.reset(new Vector(surfaceNormal));

  // Ignore contact.
  endEffectorTrajectory.ignoreContact_ = message.ignore_contact;

  // Ignore for pose adaptation.
  endEffectorTrajectory.ignoreForPoseAdaptation_ = message.ignore_for_pose_adaptation;

  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::LegMode& message, LegMode& legMode)
{
  // Limb.
  legMode.limb_ = adapter_.getLimbEnumFromLimbString(message.name);

  // Frame id. // TODO
//  foostep.frameId_ = message.target.header.frame_id;

  // Support mode.
  legMode.ignoreContact_ = !message.support_leg;

  // Duration.
  legMode.duration_ = ros::Duration(message.duration).toSec();

  // Surface normal.
  Vector surfaceNormal;
  kindr_ros::convertFromRosGeometryMsg(message.surface_normal.vector, surfaceNormal);
  legMode.surfaceNormal_.reset(new Vector(surfaceNormal));

  // Ignore for pose adaptation.
  legMode.ignoreForPoseAdaptation_ = message.ignore_for_pose_adaptation;

  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::JointTrajectory& message, JointTrajectory& jointTrajectory)
{
  // Limb.
  jointTrajectory.limb_ = adapter_.getLimbEnumFromLimbString(message.name);

  // Trajectory. // TODO
  jointTrajectory.controlSetup_[ControlLevel::Position] = false;
  jointTrajectory.controlSetup_[ControlLevel::Velocity] = false;
  jointTrajectory.controlSetup_[ControlLevel::Acceleration] = false;
  jointTrajectory.controlSetup_[ControlLevel::Effort] = false;

  jointTrajectory.jointNodeEnums_.clear();
  for (const auto& jointName : message.trajectory.joint_names) {
    jointTrajectory.jointNodeEnums_.push_back(adapter_.getJointNodeEnumFromJointNodeString(jointName));
  }

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
  kindr_ros::convertFromRosGeometryMsg(message.surface_normal.vector, surfaceNormal);
  jointTrajectory.surfaceNormal_.reset(new Vector(surfaceNormal));

  // Ignore contact.
  jointTrajectory.ignoreContact_ = message.ignore_contact;

  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::BaseAuto& message,
                                   BaseAuto& baseAuto)
{
  baseAuto.height_.reset(new double(message.height));
  baseAuto.ignoreTimingOfLegMotion_ = message.ignore_timing_of_leg_motion;
  baseAuto.averageLinearVelocity_ = message.average_linear_velocity;
  baseAuto.averageAngularVelocity_ = message.average_angular_velocity;
  baseAuto.supportMargin_ = message.support_margin;
  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::BaseTarget& message,
                                   BaseTarget& baseTarget)
{
  // Target.
  baseTarget.frameId_ = message.target.header.frame_id;
  Pose target;
  kindr_ros::convertFromRosGeometryMsg(message.target.pose, target);
  baseTarget.target_ = target;

  baseTarget.ignoreTimingOfLegMotion_ = message.ignore_timing_of_leg_motion;
  baseTarget.averageLinearVelocity_ = message.average_linear_velocity;
  baseTarget.averageAngularVelocity_ = message.average_angular_velocity;
  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::BaseTrajectory& message,
                                   BaseTrajectory& baseTrajectory)
{
  // Trajectory.
  baseTrajectory.frameIds_[ControlLevel::Position] = message.trajectory.header.frame_id;

  // We assume there is only one multi dof joint (for the base) in the message.
//  size_t nJoints = message.trajectory.joint_names.size();
//  size_t j = 0;
//  for (; j < nJoints; ++j) {
//    if (message.trajectory.joint_names[j] == "base") break;
//    if (j == nJoints - 1) return false;  // Joint name not found.
//  }

  // TODO.
  baseTrajectory.controlSetup_[ControlLevel::Position] = false;
  baseTrajectory.controlSetup_[ControlLevel::Velocity] = false;
  baseTrajectory.controlSetup_[ControlLevel::Acceleration] = false;
  baseTrajectory.controlSetup_[ControlLevel::Effort] = false;

  for (const auto& point : message.trajectory.points) {
    if (!point.transforms.empty()) baseTrajectory.controlSetup_[ControlLevel::Position] = true;
    if (!point.velocities.empty()) baseTrajectory.controlSetup_[ControlLevel::Velocity] = true;
    if (!point.accelerations.empty()) baseTrajectory.controlSetup_[ControlLevel::Acceleration] = true;
  }

  if (baseTrajectory.controlSetup_[ControlLevel::Position]) {
    baseTrajectory.values_[ControlLevel::Position] = std::vector<BaseTrajectory::ValueType>();
  }
  for (const auto& controlSetup : baseTrajectory.controlSetup_) {
    if (controlSetup.first == ControlLevel::Position || !controlSetup.second) continue;
    baseTrajectory.times_[controlSetup.first] = std::vector<BaseTrajectory::Time>();
    baseTrajectory.derivatives_[controlSetup.first] = std::vector<BaseTrajectory::DerivativeType>();
  }

  for (const auto& point : message.trajectory.points) {
    if (!point.transforms.empty()) baseTrajectory.times_[ControlLevel::Position].push_back(ros::Duration(point.time_from_start).toSec());
    if (!point.velocities.empty()) baseTrajectory.times_[ControlLevel::Velocity].push_back(ros::Duration(point.time_from_start).toSec());
    if (!point.accelerations.empty()) baseTrajectory.times_[ControlLevel::Acceleration].push_back(ros::Duration(point.time_from_start).toSec());
  }

  for (const auto& controlSetup : baseTrajectory.controlSetup_) {
    if (!controlSetup.second)continue;
    for (const auto& point : message.trajectory.points) {
      if (controlSetup.first == ControlLevel::Position && !point.transforms.empty()) {
        BaseTrajectory::ValueType pose;
        kindr_ros::convertFromRosGeometryMsg(point.transforms[0], pose);
        baseTrajectory.values_[controlSetup.first].push_back(pose);
      } else if (controlSetup.first == ControlLevel::Velocity && !point.velocities.empty()) {
//        baseTrajectory.derivatives_[controlSetup.first][j].push_back(point.velocities[j]);
      } else if (controlSetup.first == ControlLevel::Acceleration && !point.accelerations.empty()) {
//        baseTrajectory.derivatives_[controlSetup.first][j].push_back(point.accelerations[j]);
      } /*else if (controlSetup.first == ControlLevel::Effort && !point.effort.empty()) {
          baseTrajectory.derivatives_[controlSetup.first][j].push_back(point.effort[j]);
      }*/
    }
  }

  return true;
}

bool StepRosConverter::fromMessage(const free_gait_msgs::CustomCommand& message, CustomCommand& customCommand)
{
  customCommand.type_ = message.type;
  customCommand.command_ = message.command;
  customCommand.duration_ = ros::Duration(message.duration).toSec();
  return true;
}

bool StepRosConverter::toMessage(const StepQueue& stepQueue, free_gait_msgs::ExecuteStepsGoal::_steps_type& message)
{
  for (const Step& step : stepQueue.getQueue()) {
    free_gait_msgs::Step stepMessage;
    if (!toMessage(step, stepMessage)) return false;
    message.push_back(stepMessage);
  }

  return true;
}

bool StepRosConverter::toMessage(const Step& step, free_gait_msgs::Step& message)
{
  free_gait_msgs::Step& stepMessage = message;

  // ID.
  message.id = step.getId();

  // Leg motions.
  for (const auto& legMotion : step.getLegMotions()) {

    // Foostep.
    if (legMotion.second->getType() == LegMotionBase::Type::Footstep) {
      const Footstep& footstep = dynamic_cast<const Footstep&>(*(legMotion.second));
      free_gait_msgs::Footstep message;
      if (!toMessage(footstep, message)) return false;
      stepMessage.footstep.push_back(message);
    }

    // EndEffectorTajectory
    if (legMotion.second->getType() == LegMotionBase::Type::EndEffectorTrajectory) {
      const EndEffectorTrajectory& endEffectorTrajectory = dynamic_cast<const EndEffectorTrajectory&>(*(legMotion.second));
      free_gait_msgs::EndEffectorTrajectory message;
      if (!toMessage(endEffectorTrajectory, message)) return false;
      stepMessage.end_effector_trajectory.push_back(message);
    }

    // JointTrajectory
    if (legMotion.second->getType() == LegMotionBase::Type::JointTrajectory) {
      const JointTrajectory& jointTrajectory = dynamic_cast<const JointTrajectory&>(*(legMotion.second));
      free_gait_msgs::JointTrajectory message;
      if (!toMessage(jointTrajectory, message)) return false;
      stepMessage.joint_trajectory.push_back(message);
    }
  }

  // Base motion.
  if (step.hasBaseMotion()) {

    const auto& baseMotion = step.getBaseMotion();

    // Base Auto.
    if (baseMotion.getType() == BaseMotionBase::Type::Auto) {
      const BaseAuto& baseAuto = dynamic_cast<const BaseAuto&>(baseMotion);
      free_gait_msgs::BaseAuto message;
      if (!toMessage(baseAuto, message)) return false;
      stepMessage.base_auto.push_back(message);
    }

    // Base Trajectory.
    if (baseMotion.getType() == BaseMotionBase::Type::Trajectory) {
      const BaseTrajectory& baseTrajectory = dynamic_cast<const BaseTrajectory&>(baseMotion);
      free_gait_msgs::BaseTrajectory message;
      if (!toMessage(baseTrajectory, message)) return false;
      stepMessage.base_trajectory.push_back(message);
    }
  }

  return true;
}

bool StepRosConverter::toMessage(const Footstep& footstep, free_gait_msgs::Footstep& message)
{
  // Limb.
  message.name = adapter_.getLimbStringFromLimbEnum(footstep.limb_);

  // Target.
  message.target.header.frame_id = footstep.frameId_;
  kindr_ros::convertToRosGeometryMsg(footstep.target_, message.target.point);

  // Profile.
  message.profile_height = footstep.profileHeight_;
  message.profile_type = footstep.profileType_;

  // Average Velocity.
  message.average_velocity = footstep.averageVelocity_;

  // Surface normal.
  if (footstep.surfaceNormal_) {
    kindr_ros::convertToRosGeometryMsg(*(footstep.surfaceNormal_), message.surface_normal.vector);
  }

  // Ignore contact.
  message.ignore_contact = footstep.ignoreContact_;

  // Ignore for pose adaptation.
  message.ignore_for_pose_adaptation = footstep.ignoreForPoseAdaptation_;

  return true;
}

bool StepRosConverter::toMessage(const EndEffectorTrajectory& endEffectorTrajectory, free_gait_msgs::EndEffectorTrajectory& message)
{
  // Limb.
  message.name = adapter_.getLimbStringFromLimbEnum(endEffectorTrajectory.limb_);

  // Trajectory.
  message.trajectory.header.frame_id = endEffectorTrajectory.frameIds_.at(ControlLevel::Position);
  message.trajectory.points.resize(endEffectorTrajectory.times_.size());
  size_t i = 0;
  for (auto& point : message.trajectory.points) {
    point.time_from_start = ros::Duration(endEffectorTrajectory.times_[i]);

    if (endEffectorTrajectory.controlSetup_.at(ControlLevel::Position)) {
      geometry_msgs::Transform transform;
      Position position(endEffectorTrajectory.values_.at(ControlLevel::Position)[i]);
      kindr_ros::convertToRosGeometryMsg(position, transform.translation);
      point.transforms.push_back(transform);
    }

    ++i;
  }

  // Surface normal.
  if (endEffectorTrajectory.surfaceNormal_) {
    kindr_ros::convertToRosGeometryMsg(*(endEffectorTrajectory.surfaceNormal_), message.surface_normal.vector);
  }

  // Ignore contact.
  message.ignore_contact = endEffectorTrajectory.ignoreContact_;

  // Ignore for pose adaptation.
  message.ignore_for_pose_adaptation = endEffectorTrajectory.ignoreForPoseAdaptation_;

  return true;
}

bool StepRosConverter::toMessage(const JointTrajectory& jointTrajectory, free_gait_msgs::JointTrajectory& message)
{
  // Limb.
  message.name = adapter_.getLimbStringFromLimbEnum(jointTrajectory.limb_);

  // Surface normal.
  if (jointTrajectory.surfaceNormal_) {
    kindr_ros::convertToRosGeometryMsg(*(jointTrajectory.surfaceNormal_), message.surface_normal.vector);
  }

  // Trajectory.
  for (const auto& jointNode : jointTrajectory.jointNodeEnums_) {
    message.trajectory.joint_names.push_back(adapter_.getJointNodeStringFromJointNodeEnum(jointNode));
  }
  for (const auto& controlLevel : jointTrajectory.controlSetup_) {
    if (controlLevel.second) {
      size_t i = 0;
      for (const auto& time : jointTrajectory.times_.at(controlLevel.first)) {
        std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator it =
            std::find_if(message.trajectory.points.begin(), message.trajectory.points.end(),
                         [&](const trajectory_msgs::JointTrajectoryPoint& point) -> bool {return point.time_from_start.toSec() >= time;});
        if (it == message.trajectory.points.end() || it->time_from_start.toSec() != time) {
          it = message.trajectory.points.insert(it, trajectory_msgs::JointTrajectoryPoint());
          it->time_from_start = ros::Duration(time);
        }
        if (controlLevel.first == ControlLevel::Position)
          it->positions = jointTrajectory.values_.at(controlLevel.first)[i];
        else if (controlLevel.first == ControlLevel::Velocity)
          it->velocities = jointTrajectory.values_.at(controlLevel.first)[i];
        else if (controlLevel.first == ControlLevel::Acceleration)
          it->accelerations = jointTrajectory.values_.at(controlLevel.first)[i];
        else if (controlLevel.first == ControlLevel::Effort)
          it->effort = jointTrajectory.values_.at(controlLevel.first)[i];
        i++;
      }
    }
  }

  // Ignore contact.
  message.ignore_contact = jointTrajectory.ignoreContact_;

  return true;
}

bool StepRosConverter::toMessage(const BaseAuto& baseAuto, free_gait_msgs::BaseAuto& message)
{
  if (baseAuto.height_) message.height = *(baseAuto.height_);
  message.ignore_timing_of_leg_motion = baseAuto.ignoreTimingOfLegMotion_;
  message.average_linear_velocity = baseAuto.averageLinearVelocity_;
  message.average_angular_velocity = baseAuto.averageAngularVelocity_;
  message.support_margin = baseAuto.supportMargin_;
  return true;
}

bool StepRosConverter::toMessage(const BaseTrajectory& baseTrajectory, free_gait_msgs::BaseTrajectory& message)
{
  message.trajectory.header.frame_id = baseTrajectory.frameIds_.at(ControlLevel::Position);
  message.trajectory.points.resize(baseTrajectory.values_.at(ControlLevel::Position).size());
  size_t i = 0;
  for (auto& point : message.trajectory.points) {
    if (baseTrajectory.controlSetup_.at(ControlLevel::Position)) {
      point.time_from_start = ros::Duration(baseTrajectory.times_.at(ControlLevel::Position)[i]);
      geometry_msgs::Transform transform;
      BaseTrajectory::ValueType pose(baseTrajectory.values_.at(ControlLevel::Position)[i]);
      kindr_ros::convertToRosGeometryMsg(pose, transform);
      point.transforms.push_back(transform);
    }
    ++i;
  }

  return true;
}

} /* namespace */
