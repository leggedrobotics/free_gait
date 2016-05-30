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
  virtual bool resetExtrasWithRobot(const StepQueue& stepQueue, State& state) = 0;
  virtual bool updateExtrasBefore(const StepQueue& stepQueue, State& state) = 0;
  virtual bool updateExtrasAfter(const StepQueue& stepQueue, State& state) = 0;

  //! State independent functions.
  virtual const std::string& getWorldFrameId() const = 0;
  virtual const std::vector<LimbEnum>& getLimbs() const = 0;
  virtual const std::vector<BranchEnum>& getBranches() const = 0;
  virtual LimbEnum getLimbEnumFromLimbString(const std::string& limb) const = 0;
  virtual std::string getLimbStringFromLimbEnum(const LimbEnum& limb) const = 0;
  virtual bool getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
      const Position& positionBaseToFootInBaseFrame, const LimbEnum& limb,
      JointPositionsLeg& jointPositions) const = 0;
  virtual Position getPositionBaseToFootInBaseFrame(const LimbEnum& limb,
                                                    const JointPositionsLeg& jointPositions) const = 0;

  //! Reading state of real robot.
  virtual bool isExecutionOk() const = 0;
  virtual bool isLegGrounded(const LimbEnum& limb) const = 0;
  virtual JointPositionsLeg getJointPositionsForLimb(const LimbEnum& limb) const = 0;
  virtual JointPositions getAllJointPositions() const = 0;
  virtual JointVelocitiesLeg getJointVelocitiesForLimb(const LimbEnum& limb) const = 0;
  virtual JointVelocities getAllJointVelocities() const = 0;
  virtual JointAccelerationsLeg getJointAccelerationsForLimb(const LimbEnum& limb) const = 0;
  virtual JointAccelerations getAllJointAccelerations() const = 0;
  virtual JointEffortsLeg getJointEffortsForLimb(const LimbEnum& limb) const = 0;
  virtual JointEfforts getAllJointEfforts() const = 0;
  virtual Position getPositionWorldToBaseInWorldFrame() const = 0;
  virtual RotationQuaternion getOrientationWorldToBase() const = 0;
  virtual Position getPositionBaseToFootInBaseFrame(const LimbEnum& limb) const = 0;
  virtual Position getPositionWorldToFootInWorldFrame(const LimbEnum& limb) const = 0;
  virtual Pose getFrameTransform(const std::string& frameId) const = 0;
  virtual ControlSetup getControlSetup(const BranchEnum& branch) const = 0;
  virtual ControlSetup getControlSetup(const LimbEnum& limb) const = 0;

  //! Depending on state of real robot.
  virtual bool frameIdExists(const std::string& frameId) const;
  virtual Position transformPosition(const std::string& inputFrameId,
                                     const std::string& outputFrameId, const Position& position) const;
  virtual RotationQuaternion transformOrientation(const std::string& inputFrameId,
                                                  const std::string& outputFrameId,
                                                  const RotationQuaternion& orientation) const;
  virtual Pose transformPose(const std::string& inputFrameId, const std::string& outputFrameId,
                             const Pose& pose) const;
};

} /* namespace free_gait */
