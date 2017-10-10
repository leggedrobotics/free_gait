/*
 * AdapterDummy.hpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include "free_gait_core/executor/AdapterBase.hpp"

#include <memory>

namespace free_gait {

class AdapterDummy : public AdapterBase
{
 public:
  AdapterDummy();
  virtual ~AdapterDummy();

  //! Copying data from real robot to free gait state.
  virtual bool resetExtrasWithRobot(const StepQueue& stepQueue, State& state);
  virtual bool updateExtrasBefore(const StepQueue& stepQueue, State& state);
  virtual bool updateExtrasAfter(const StepQueue& stepQueue, State& state);

  //! State independent functions.
  virtual const std::string& getWorldFrameId() const;
  virtual const std::string& getBaseFrameId() const;
  virtual const std::vector<LimbEnum>& getLimbs() const;
  virtual const std::vector<BranchEnum>& getBranches() const;
  virtual LimbEnum getLimbEnumFromLimbString(const std::string& limb) const;
  virtual std::string getLimbStringFromLimbEnum(const LimbEnum& limb) const;
  virtual std::string getBaseString() const;
  virtual JointNodeEnum getJointNodeEnumFromJointNodeString(const std::string& jointNode) const;
  virtual std::string getJointNodeStringFromJointNodeEnum(const JointNodeEnum& jointNode) const;
  virtual bool getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
      const Position& positionBaseToFootInBaseFrame, const LimbEnum& limb,
      JointPositionsLeg& jointPositions) const;
  virtual Position getPositionBaseToFootInBaseFrame(const LimbEnum& limb,
                                                    const JointPositionsLeg& jointPositions) const;
  virtual Position getPositionBaseToHipInBaseFrame(const LimbEnum& limb) const;

  //! Reading state of real robot.
  virtual bool isExecutionOk() const;
  virtual bool isLegGrounded(const LimbEnum& limb) const;
  virtual JointPositionsLeg getJointPositionsForLimb(const LimbEnum& limb) const;
  virtual JointPositions getAllJointPositions() const;
  virtual JointVelocitiesLeg getJointVelocitiesForLimb(const LimbEnum& limb) const;
  virtual JointVelocities getAllJointVelocities() const;
  virtual JointAccelerationsLeg getJointAccelerationsForLimb(const LimbEnum& limb) const;
  virtual JointAccelerations getAllJointAccelerations() const;
  virtual JointEffortsLeg getJointEffortsForLimb(const LimbEnum& limb) const;
  virtual JointEfforts getAllJointEfforts() const;
  virtual Position getPositionWorldToBaseInWorldFrame() const;
  virtual RotationQuaternion getOrientationBaseToWorld() const;
  virtual LinearVelocity getLinearVelocityBaseInWorldFrame() const;
  virtual LocalAngularVelocity getAngularVelocityBaseInBaseFrame() const;
  virtual LinearAcceleration getLinearAccelerationBaseInWorldFrame() const;
  virtual AngularAcceleration getAngularAccelerationBaseInBaseFrame() const;
  virtual Position getPositionBaseToFootInBaseFrame(const LimbEnum& limb) const;
  virtual Position getPositionWorldToFootInWorldFrame(const LimbEnum& limb) const;
  virtual Position getCenterOfMassInWorldFrame() const;

  /*!
   * Transform is frameId to world => C_IF.
   * @param frameId
   * @return
   */
  virtual void getAvailableFrameTransforms(std::vector<std::string>& frameTransforms) const;
  virtual Pose getFrameTransform(const std::string& frameId) const;
  virtual ControlSetup getControlSetup(const BranchEnum& branch) const;
  virtual ControlSetup getControlSetup(const LimbEnum& limb) const;

  //! Depending on state of real robot.
  virtual JointVelocitiesLeg getJointVelocitiesFromEndEffectorLinearVelocityInWorldFrame(
      const LimbEnum& limb, const LinearVelocity& endEffectorLinearVelocityInWorldFrame) const;
  virtual LinearVelocity getEndEffectorLinearVelocityFromJointVelocities(const LimbEnum& limb,
                                                                         const JointVelocitiesLeg& jointVelocities,
                                                                         const std::string& frameId) const;
  virtual JointAccelerationsLeg getJointAccelerationsFromEndEffectorLinearAccelerationInWorldFrame(
      const LimbEnum& limb, const LinearAcceleration& endEffectorLinearAccelerationInWorldFrame) const;

  //! Hook to write data to internal robot representation from state.
  virtual bool setInternalDataFromState(const State& state) const;

  const State& getState() const;

 private:
  std::unique_ptr<State> state_;
  std::vector<LimbEnum> limbs_;
  std::vector<BranchEnum> branches_;
  const std::string worldFrameId_;
  const std::string baseFrameId_;
};

} /* namespace free_gait */
