/*
 * AdapterBase.hpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/executor/State.hpp"
#include "free_gait_core/step/StepQueue.hpp"
#include <memory>
#include <string>

namespace free_gait {

class State;
class StepQueue;

class AdapterBase
{
 public:
  AdapterBase();
  virtual ~AdapterBase();

  //! Copying data from real robot to free gait state.
  virtual bool updateExtras(const StepQueue& stepQueue, State& state) const = 0;

  //! State independent functions.
  virtual const std::vector<LimbEnum>& getLimbs() const = 0;
  virtual const std::vector<BranchEnum>& getBranches() const = 0;
  virtual LimbEnum getLimbEnumFromLimbString(const std::string& limb) const = 0;
  virtual bool getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
      const Position& positionBaseToFootInBaseFrame, const LimbEnum& limb,
      JointPositions& legJoints) const = 0;
  virtual Position getPositionBaseToFootInBaseFrame(const LimbEnum& limb,
                                                    const JointPositions& jointPositions) const = 0;

  //! Reading state of real robot.
  virtual bool isLegGrounded(const LimbEnum& limb) const = 0;
  virtual JointPositions getJointPositions(const LimbEnum& limb) const = 0;
  virtual JointPositions getAllJointPositions() const = 0;
  virtual JointVelocities getJointVelocities(const LimbEnum& limb) const = 0;
  virtual JointVelocities getAllJointVelocities() const = 0;
  virtual JointAccelerations getJointAccelerations(const LimbEnum& limb) const = 0;
  virtual JointAccelerations getAllJointAccelerations() const = 0;
  virtual JointEfforts getJointEfforts(const LimbEnum& limb) const = 0;
  virtual JointEfforts getAllJointEfforts() const = 0;
  virtual const Position& getPositionWorldToBaseInWorldFrame() const = 0;
  virtual const RotationQuaternion& getOrientationWorldToBase() const = 0;
  virtual Position getPositionBaseToFootInBaseFrame(const LimbEnum& limb) const = 0;
  virtual Position getPositionWorldToFootInWorldFrame(const LimbEnum& limb) const = 0;
  virtual ControlSetup getControlSetup(const BranchEnum& branch) const = 0;
  virtual ControlSetup getControlSetup(const LimbEnum& limb) const = 0;
};

} /* namespace free_gait */
