/*
 * RosControllerInterfaceJointTrajectories.cpp
 *
 *  Created on: March 8, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco_joint_trajectories_ros/RosControllerInterfaceJointTrajectories.hpp"

namespace loco_joint_trajectories_ros {

RosControllerInterfaceJointTrajectories::RosControllerInterfaceJointTrajectories()
    : loco_ros::RosControllerInterfaceBase()
{

}

RosControllerInterfaceJointTrajectories::~RosControllerInterfaceJointTrajectories()
{

}

void RosControllerInterfaceJointTrajectories::updateMessages()
{
  stepActionServer_->update();
}

void RosControllerInterfaceJointTrajectories::publish()
{

}

void RosControllerInterfaceJointTrajectories::initializePublishers()
{

}

void RosControllerInterfaceJointTrajectories::initializeStepActionServer(
    std::shared_ptr<loco::StepQueue> stepQueue, std::shared_ptr<loco::StepCompleter> stepCompleter, std::shared_ptr<loco::LegGroup> legs,
    std::shared_ptr<loco::TorsoBase> torso)
{
  stepActionServer_ = std::unique_ptr<loco_ros::StepActionServer>(new loco_ros::StepActionServer(nodeHandle_, "step", stepQueue, stepCompleter, legs, torso));
}

} /* namespace */
