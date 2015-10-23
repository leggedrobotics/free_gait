/*
 * ExecutorState.hpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/TypeDefs.hpp"
#include "quadruped_model/QuadrupedState.hpp"

// STD
#include <vector>

namespace free_gait {

class ExecutorState : quadruped_model::QuadrupedState
{
 public:
  ExecutorState();
  virtual ~ExecutorState();
  friend std::ostream& operator<<(std::ostream& out, const ExecutorState& state);

  virtual void initialize(const std::vector<LimbEnum>& limbs, const std::vector<BranchEnum>& branches);

  bool isSupportLeg(const LimbEnum& limb) const;
  const std::unordered_map<LimbEnum, bool, EnumClassHash>& getIsSupportLegs() const;
  void setSupportLeg(const LimbEnum& limb, bool isSupportLeg);

  bool isIgnoreContact(const LimbEnum& limb) const;
  const std::unordered_map<LimbEnum, bool, EnumClassHash>& getIsIgnoreContact() const;
  void setIgnoreContact(const LimbEnum& limb, bool ignoreContact);

 private:
  // TODO Extend QuadrupedState class with:
  JointEfforts jointTorques_;
  std::unordered_map<LimbEnum, bool, EnumClassHash> isSupportLegs_;

  // Free gait specific.
  std::unordered_map<BranchEnum, ControlSetup, EnumClassHash> controlSetups_;
  LinearAcceleration linearAccelerationBaseInWorldFrame_;
  AngularAcceleration angularAccelerationBaseInBaseFrame_;
  Force netForceOnBaseInBaseFrame_;
  Torque netTorqueOnBaseInBaseFrame_;
  std::unordered_map<LimbEnum, bool, EnumClassHash> ignoreContact_;
};

} /* namespace */
