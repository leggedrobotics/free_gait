/*
 * JointTrajectoriesRos.hpp
 *
 *  Created on: Feb 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "loco_joint_trajectories/JointTrajectories.hpp"
#include "loco_joint_trajectories_ros/RosControllerInterfaceJointTrajectories.hpp"

namespace loco_joint_trajectories_ros {

class JointTrajectoriesRos : public loco_joint_trajectories::JointTrajectories, public RosControllerInterfaceJointTrajectories
{
 public:
  JointTrajectoriesRos();
  virtual ~JointTrajectoriesRos();

  virtual bool advance(double dt);
  virtual bool initialize(double dt);
};

} /* namespace */
