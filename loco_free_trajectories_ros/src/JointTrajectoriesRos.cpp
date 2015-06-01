/*
 * JointTrajectoriesRos.cpp
 *
 *  Created on: Mar 9, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco_joint_trajectories_ros/JointTrajectoriesRos.hpp"

namespace loco_joint_trajectories_ros {

JointTrajectoriesRos::JointTrajectoriesRos() :
    JointTrajectories("JointTrajectoriesRos"),
    RosControllerInterfaceJointTrajectories()
{

}

JointTrajectoriesRos::~JointTrajectoriesRos()
{

}

bool JointTrajectoriesRos::initialize(double dt) {
  JointTrajectories::initialize(dt);
  initializePublishers();
  initializeStepActionServer(stepQueue_, stepCompleter_, legs_, torso_);
  return true;
}

bool JointTrajectoriesRos::advance(double dt) {
  JointTrajectories::advance(dt);
  updateMessages();
  publish();

  return true;
}

} /* namespace */
