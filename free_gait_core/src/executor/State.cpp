/*
 * State.cpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "free_gait_core/executor/State.hpp"

namespace free_gait {

State::State()
: QuadrupedState()
{
}

State::~State()
{
}

void State::initialize(const std::vector<LimbEnum>& limbs, const std::vector<BranchEnum>& branches)
{
  limbs_ = limbs;
  for (const auto& limb : limbs_) {
    isSupportLegs_[limb] = false;
    ignoreContact_[limb] = false;
    ignoreForPoseAdaptation_[limb] = false;
  }
}

const std::vector<LimbEnum>& State::getLimbs() const
{
 return limbs_;
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

bool State::isIgnoreForPoseAdaptation(const LimbEnum& limb) const
{
  return ignoreForPoseAdaptation_.at(limb);
}

void State::setIgnoreForPoseAdaptation(const LimbEnum& limb, bool ignorePoseAdaptation)
{
  ignoreForPoseAdaptation_[limb] = ignorePoseAdaptation;
}

} /* namespace free_gait */

