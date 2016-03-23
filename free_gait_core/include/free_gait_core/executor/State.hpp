/*
 * State.hpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/TypeDefs.hpp"
#include <quadruped_model/QuadrupedState.hpp>

// STD
#include <vector>
#include <iostream>

namespace free_gait {

class State : public quadruped_model::QuadrupedState
{
 public:
  State();
  virtual ~State();

  virtual void initialize(const std::vector<LimbEnum>& limbs, const std::vector<BranchEnum>& branches);

  bool getRobotExecutionStatus() const;
  void setRobotExecutionStatus(bool robotExecutionStatus);

  bool isSupportLeg(const LimbEnum& limb) const;
  void setSupportLeg(const LimbEnum& limb, bool isSupportLeg);
  unsigned int getNumberOfSupportLegs() const;

  bool isIgnoreContact(const LimbEnum& limb) const;
  void setIgnoreContact(const LimbEnum& limb, bool ignoreContact);

  bool hasSurfaceNormal(const LimbEnum& limb) const;
  const Vector& getSurfaceNormal(const LimbEnum& limb) const;
  void setSurfaceNormal(const LimbEnum& limb, const Vector& surfaceNormal);
  void removeSurfaceNormal(const LimbEnum& limb);

  bool isIgnoreForPoseAdaptation(const LimbEnum& limb) const;
  void setIgnoreForPoseAdaptation(const LimbEnum& limb, bool ignorePoseAdaptation);

  const free_gait::JointPositions getJointPositions(const LimbEnum& limb) const;
  void setJointPositions(const LimbEnum& limb, const free_gait::JointPositions& jointPositions);
  void setAllJointPositions(const JointPositions& jointPositions);

  const free_gait::JointVelocities getJointVelocities(const LimbEnum& limb) const;
  void setJointVelocities(const LimbEnum& limb, const free_gait::JointVelocities& jointVelocities);
  void setAllJointVelocities(const JointVelocities& jointVelocities);

  const free_gait::JointAccelerations getJointAccelerations(const LimbEnum& limb) const;
  const free_gait::JointAccelerations& getAllJointAccelerations() const;
  void setJointAccelerations(const LimbEnum& limb, const JointAccelerations& jointAccelerations);
  void setAllJointAccelerations(const JointAccelerations& jointAccelerations);

  const JointEfforts getJointEfforts(const LimbEnum& limb) const;
  const JointEfforts& getAllJointEfforts() const;
  void setJointEfforts(const LimbEnum& limb, const JointEfforts& jointEfforts);
  void setAllJointEfforts(const JointEfforts& jointEfforts);

  const ControlSetup& getControlSetup(const BranchEnum& branch) const;
  const ControlSetup& getControlSetup(const LimbEnum& limb) const;
  bool isControlSetupEmpty(const BranchEnum& branch) const;
  bool isControlSetupEmpty(const LimbEnum& limb) const;
  void setControlSetup(const BranchEnum& branch, const ControlSetup& controlSetup);
  void setControlSetup(const LimbEnum& limb, const ControlSetup& controlSetup);
  void setEmptyControlSetup(const BranchEnum& branch);
  void setEmptyControlSetup(const LimbEnum& limb);

  friend std::ostream& operator << (std::ostream& out, const State& state);

 private:
  // TODO Extend QuadrupedState class with:
  JointEfforts jointEfforts_;
  JointAccelerations jointAccelerations_;
  LinearAcceleration linearAccelerationBaseInWorldFrame_;
  AngularAcceleration angularAccelerationBaseInBaseFrame_;

  // Free gait specific.
  std::unordered_map<BranchEnum, ControlSetup, EnumClassHash> controlSetups_;
  Force netForceOnBaseInBaseFrame_;
  Torque netTorqueOnBaseInBaseFrame_;
  std::unordered_map<LimbEnum, bool, EnumClassHash> isSupportLegs_;
  std::unordered_map<LimbEnum, bool, EnumClassHash> ignoreContact_;
  std::unordered_map<LimbEnum, bool, EnumClassHash> ignoreForPoseAdaptation_;
  std::unordered_map<LimbEnum, Vector, EnumClassHash> surfaceNormals_;
  bool robotExecutionStatus_;
};

} /* namespace */
