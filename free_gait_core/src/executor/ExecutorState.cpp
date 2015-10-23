/*
 * ExecutorState.cpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "free_gait_core/executor/ExecutorState.hpp"

namespace free_gait {

ExecutorState::ExecutorState()
: QuadrupedState()
{
}

ExecutorState::~ExecutorState()
{
}

void ExecutorState::initialize(const std::vector<LimbEnum>& limbs,
                               const std::vector<BranchEnum>& branches)
{
  for (const auto& limb : limbs) {
    isSupportLegs_[limb] = false;
    ignoreContact_[limb] = false;
  }
}

bool ExecutorState::isSupportLeg(const LimbEnum& limb) const
{
  return isSupportLegs_.at(limb);
}

const std::unordered_map<LimbEnum, bool, EnumClassHash>& ExecutorState::getIsSupportLegs() const
{
  return isSupportLegs_;
}

void ExecutorState::setSupportLeg(const LimbEnum& limb, bool isSupportLeg)
{
  isSupportLegs_[limb] = isSupportLeg;
}

bool ExecutorState::isIgnoreContact(const LimbEnum& limb) const
{
  return ignoreContact_.at(limb);
}

const std::unordered_map<LimbEnum, bool, EnumClassHash>& ExecutorState::getIsIgnoreContact() const
{
  return ignoreContact_;
}

void ExecutorState::setIgnoreContact(const LimbEnum& limb, bool ignoreContact)
{
  ignoreContact_[limb] = ignoreContact;
}

} /* namespace free_gait */

