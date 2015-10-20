/*
 * StepExecutor.cpp
 *
 *  Created on: Oct 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/step/StepExecutor.hpp>

namespace free_gait {

StepExecutor::StepExecutor(std::shared_ptr<StepCompleter> completer)
    : completer_(completer)
{
  // TODO Auto-generated constructor stub

}

StepExecutor::~StepExecutor()
{
  // TODO Auto-generated destructor stub
}

bool StepExecutor::advance(double dt)
{
  if (queue_.empty()) {
    return true;
  }

  // TODO How about first step?
  // TODO Update start position.
  if (queue_.getCurrentStep().isApproachingEndOfStep() && queue_.size() > 1) {
    if (!queue_.getNextStep().isComplete()) {
      completer_->complete(queue_.getNextStep());
    }
  }

  queue_.advance(dt);
  return true;
}

StepQueue& StepExecutor::getQueue()
{
  return queue_;
}

void StepExecutor::initializeBasePose(const Pose& pose)
{
  basePose_ = pose;
}

const Pose& StepExecutor::getBasePose()
{
  return basePose_;
}

void StepExecutor::initializeJointPositions(LimbEnum limb, const JointPositions& jointPositions)
{
  jointPositions_[limb] = jointPositions;
}

bool StepExecutor::isJointPositionsAvailable(LimbEnum limb) const
{
  std::unordered_map<quadruped_model::LimbEnum, JointPositions, robotUtils::EnumClassHash>::const_iterator it =
      jointPositions_.find(limb);

  if (it == jointPositions_.end()) return false;
  return true;
}

const JointPositions& StepExecutor::getJointPositions(LimbEnum limb) const
{
  return jointPositions_.at(limb);
}

void StepExecutor::initializeJointVelocities(LimbEnum limb, const JointVelocities& jointVelocities)
{
  jointVelocities_[limb] = jointVelocities;
}

bool StepExecutor::isJointVelocitiesAvailable(LimbEnum limb) const
{
  std::unordered_map<quadruped_model::LimbEnum, JointVelocities, robotUtils::EnumClassHash>::const_iterator it =
      jointVelocities_.find(limb);

  if (it == jointVelocities_.end()) return false;
  return true;
}

const JointVelocities& StepExecutor::getJointVelocities(LimbEnum limb) const
{
  return jointVelocities_.at(limb);
}

void StepExecutor::initializeJointTorques(LimbEnum limb, const JointTorques& jointTorques)
{
  jointTorques_[limb] = jointTorques;
}

bool StepExecutor::isJointTorquesAvailable(LimbEnum limb) const
{
  std::unordered_map<quadruped_model::LimbEnum, JointTorques, robotUtils::EnumClassHash>::const_iterator it =
      jointTorques_.find(limb);

  if (it == jointTorques_.end()) return false;
  return true;
}

const JointTorques& StepExecutor::getJointTorques(LimbEnum limb) const
{
  return jointTorques_.at(limb);
}

void StepExecutor::initializeSupportLeg(LimbEnum limb, bool support)
{
  isSupportLegs_[limb] = support;
}

bool StepExecutor::isSupportLeg(LimbEnum limb) const
{
  return isSupportLegs_.at(limb);
}

} /* namespace free_gait */
