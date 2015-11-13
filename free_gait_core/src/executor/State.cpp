/*
 * State.cpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/executor/State.hpp>
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

void State::initialize(const std::vector<LimbEnum>& limbs)
{
  for (const auto& limb : limbs) {
    isSupportLegs_[limb] = false;
    ignoreContact_[limb] = false;
    ignoreForPoseAdaptation_[limb] = false;
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
  // TODO This is not nice.
  unsigned int startIndex = 3 * quadruped_model::getLimbUIntFromLimbEnum(limb);
  return JointPositions(quadruped_model::QuadrupedState::getJointPositions().vector().segment<3>(startIndex));
//  return quadruped_model::QuadrupedState::getJointPositions().vector().segment<3>(startIndex);
}

void State::setJointPositions(const LimbEnum& limb, const JointPositions& jointPositions)
{
  // TODO This is not nice.
  unsigned int startIndex = 3 * quadruped_model::getLimbUIntFromLimbEnum(limb);
  for (unsigned int i = 0; i < 3; ++i) {
    quadruped_model::QuadrupedState::getJointPositions()(startIndex + i) = jointPositions(i);
  }
}

void State::setAllJointPositions(const JointPositions& jointPositions)
{
  quadruped_model::QuadrupedState::setJointPositions(quadruped_model::JointPositions(jointPositions.vector()));
}

void State::setAllJointVelocities(const JointVelocities& jointVelocities)
{
  quadruped_model::QuadrupedState::setJointVelocities(quadruped_model::JointVelocities(jointVelocities.vector()));
}

} /* namespace free_gait */

