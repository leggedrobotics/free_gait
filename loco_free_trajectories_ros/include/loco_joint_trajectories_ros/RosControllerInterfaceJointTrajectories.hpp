/*
 * RosControllerInterfaceJointTrajectories.hpp
 *
 *  Created on: Feb 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Loco
#include "loco/common/StepQueue.hpp"
#include "loco/common/StepCompleter.hpp"
#include "loco_ros/ros_controller_interface/RosControllerInterfaceBase.hpp"
#include "loco_ros/StepActionServer.hpp"

// STD
#include <memory>

namespace loco_joint_trajectories_ros {

class RosControllerInterfaceJointTrajectories : public loco_ros::RosControllerInterfaceBase
{
 public:
  RosControllerInterfaceJointTrajectories();
  virtual ~RosControllerInterfaceJointTrajectories();

  virtual void initializePublishers();
  void initializeStepActionServer(std::shared_ptr<loco::StepQueue> stepQueue,
                                  std::shared_ptr<loco::StepCompleter> stepCompleter,
                                  std::shared_ptr<loco::LegGroup> legs,
                                  std::shared_ptr<loco::TorsoBase> torso);
  virtual void updateMessages();
  virtual void publish();

 private:
  std::unique_ptr<loco_ros::StepActionServer> stepActionServer_;
};

} /* namespace */
