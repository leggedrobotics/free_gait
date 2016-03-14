/*
 * State.cpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/executor/State.hpp>
#include <free_gait_core/TypePrints.hpp>
#include <quadruped_model/common/topology_conversions.hpp>

namespace free_gait {

State::State()
    : QuadrupedState(),
      robotExecutionStatus_(false)
{
}

State::~State()
{
}

void State::initialize(const std::vector<LimbEnum>& limbs, const std::vector<BranchEnum>& branches)
{
  for (const auto& limb : limbs) {
    isSupportLegs_[limb] = false;
    ignoreContact_[limb] = false;
    ignoreForPoseAdaptation_[limb] = false;
  }

  for (const auto& branch : branches) {
    setEmptyControlSetup(branch);
  }
}

bool State::getRobotExecutionStatus() const
{
  return robotExecutionStatus_;
}

void State::setRobotExecutionStatus(bool robotExecutionStatus)
{
  robotExecutionStatus_ = robotExecutionStatus;
}

bool State::isSupportLeg(const LimbEnum& limb) const
{
  return isSupportLegs_.at(limb);
}

void State::setSupportLeg(const LimbEnum& limb, bool isSupportLeg)
{
  isSupportLegs_[limb] = isSupportLeg;
}

unsigned int State::getNumberOfSupportLegs() const
{
  unsigned int nLegs = 0;
  for (const auto& supportLeg : isSupportLegs_) {
    if (supportLeg.second) ++nLegs;
  }
  return nLegs;
}

bool State::isIgnoreContact(const LimbEnum& limb) const
{
  return ignoreContact_.at(limb);
}

void State::setIgnoreContact(const LimbEnum& limb, bool ignoreContact)
{
  ignoreContact_[limb] = ignoreContact;
}

bool State::hasSurfaceNormal(const LimbEnum& limb) const
{
  return (surfaceNormals_.count(limb) > 0);
}

const Vector& State::getSurfaceNormal(const LimbEnum& limb) const
{
  return surfaceNormals_.at(limb);
}

void State::setSurfaceNormal(const LimbEnum& limb, const Vector& surfaceNormal)
{
  surfaceNormals_[limb] = surfaceNormal;
}

void State::removeSurfaceNormal(const LimbEnum& limb)
{
  surfaceNormals_.erase(limb);
}

bool State::isIgnoreForPoseAdaptation(const LimbEnum& limb) const
{
  return ignoreForPoseAdaptation_.at(limb);
}

void State::setIgnoreForPoseAdaptation(const LimbEnum& limb, bool ignorePoseAdaptation)
{
  ignoreForPoseAdaptation_[limb] = ignorePoseAdaptation;
}

const JointPositions State::getJointPositions(const LimbEnum& limb) const
{
  unsigned int startIndex = 3 * quadruped_model::getLimbUIntFromLimbEnum(limb);
  return JointPositions(quadruped_model::QuadrupedState::getJointPositions().vector().segment<3>(startIndex));
}

void State::setJointPositions(const LimbEnum& limb, const JointPositions& jointPositions)
{
  unsigned int startIndex = 3 * quadruped_model::getLimbUIntFromLimbEnum(limb);
  for (unsigned int i = 0; i < 3; ++i) {
    quadruped_model::QuadrupedState::getJointPositions()(startIndex + i) = jointPositions(i);
  }
}

void State::setAllJointPositions(const JointPositions& jointPositions)
{
  quadruped_model::QuadrupedState::setJointPositions(quadruped_model::JointPositions(jointPositions.vector()));
}

const JointVelocities State::getJointVelocities(const LimbEnum& limb) const
{
  unsigned int startIndex = 3 * quadruped_model::getLimbUIntFromLimbEnum(limb);
  return JointVelocities(quadruped_model::QuadrupedState::getJointVelocities().vector().segment<3>(startIndex));
}

void State::setJointVelocities(const LimbEnum& limb, const JointVelocities& jointVelocities)
{
  unsigned int startIndex = 3 * quadruped_model::getLimbUIntFromLimbEnum(limb);
  for (unsigned int i = 0; i < 3; ++i) {
    quadruped_model::QuadrupedState::getJointVelocities()(startIndex + i) = jointVelocities(i);
  }
}

void State::setAllJointVelocities(const JointVelocities& jointVelocities)
{
  quadruped_model::QuadrupedState::setJointVelocities(quadruped_model::JointVelocities(jointVelocities.vector()));
}

const JointAccelerations State::getJointAccelerations(const LimbEnum& limb) const
{
  unsigned int startIndex = 3 * quadruped_model::getLimbUIntFromLimbEnum(limb);
  return JointAccelerations(jointAccelerations_.vector().segment<3>(startIndex));
}

const JointAccelerations& State::getAllJointAccelerations() const
{
  return jointAccelerations_;
}

void State::setJointAccelerations(const LimbEnum& limb, const JointAccelerations& jointAccelerations)
{
  unsigned int startIndex = 3 * quadruped_model::getLimbUIntFromLimbEnum(limb);
  for (unsigned int i = 0; i < 3; ++i) {
    jointAccelerations_(startIndex + i) = jointAccelerations(i);
  }
}

void State::setAllJointAccelerations(const JointAccelerations& jointAccelerations)
{
  jointAccelerations_ = jointAccelerations;
}

const JointEfforts State::getJointEfforts(const LimbEnum& limb) const
{
  unsigned int startIndex = 3 * quadruped_model::getLimbUIntFromLimbEnum(limb);
  return JointEfforts(getAllJointEfforts().vector().segment<3>(startIndex));
}

const JointEfforts& State::getAllJointEfforts() const
{
  return jointEfforts_;
}

void State::setJointEfforts(const LimbEnum& limb, const JointEfforts& jointEfforts)
{
  unsigned int startIndex = 3 * quadruped_model::getLimbUIntFromLimbEnum(limb);
  for (unsigned int i = 0; i < 3; ++i) { // TODO Can we do block operation?
    jointEfforts_(startIndex + i) = jointEfforts(i);
  }
}

void State::setAllJointEfforts(const JointEfforts& jointEfforts)
{
  jointEfforts_ = jointEfforts;
}

const ControlSetup& State::getControlSetup(const BranchEnum& branch) const
{
  return controlSetups_.at(branch);
}

const ControlSetup& State::getControlSetup(const LimbEnum& limb) const
{
  return controlSetups_.at(quadruped_model::getBranchEnumFromLimbEnum(limb));
}

bool State::isControlSetupEmpty(const BranchEnum& branch) const
{
  for (const auto& level : controlSetups_.at(branch)) {
    if (level.second) return false;
  }
  return true;
}

bool State::isControlSetupEmpty(const LimbEnum& limb) const
{
  return isControlSetupEmpty(quadruped_model::getBranchEnumFromLimbEnum(limb));
}

void State::setControlSetup(const BranchEnum& branch, const ControlSetup& controlSetup)
{
  controlSetups_[branch] = controlSetup;
}

void State::setControlSetup(const LimbEnum& limb, const ControlSetup& controlSetup)
{
  controlSetups_[quadruped_model::getBranchEnumFromLimbEnum(limb)] = controlSetup;
}

void State::setEmptyControlSetup(const BranchEnum& branch)
{
  ControlSetup emptyControlSetup;
  emptyControlSetup[ControlLevel::Position] = false;
  emptyControlSetup[ControlLevel::Velocity] = false;
  emptyControlSetup[ControlLevel::Acceleration] = false;
  emptyControlSetup[ControlLevel::Effort] = false;
  controlSetups_[branch] = emptyControlSetup;
}

void State::setEmptyControlSetup(const LimbEnum& limb)
{
  setEmptyControlSetup(quadruped_model::getBranchEnumFromLimbEnum(limb));
}

std::ostream& operator<<(std::ostream& out, const State& state)
{
  out << "Support legs: " << state.isSupportLegs_ << std::endl;
  out << "Ignore contact: " << state.ignoreContact_ << std::endl;
  out << "Ignore for pose adaptation: " << state.ignoreForPoseAdaptation_ << std::endl;
  out << "Control setup:" << std::endl;
  for (const auto& controlSetup : state.controlSetups_) {
    out << controlSetup.first << ": ";
    for (const auto& controlLevel : controlSetup.second) {
      if (controlLevel.second) out << controlLevel.first << ", ";
    }
    out << std::endl;
  }
  return out;
}

} /* namespace free_gait */

