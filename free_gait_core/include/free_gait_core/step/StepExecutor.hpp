/*
 * StepExecutor.hpp
 *
 *  Created on: Oct 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/step/StepQueue.hpp"
#include "free_gait_core/step/StepCompleter.hpp"
#include "free_gait_core/TypeDefs.hpp"

// Robot model
#include "quadruped_model/quadruped_model.hpp"

// Robot utils
#include "robotUtils/containers/MultiKeyContainer.hpp"

// STD
#include <memory>

namespace free_gait {

class StepExecutor
{
 public:
  StepExecutor(std::shared_ptr<StepCompleter> completer);
  virtual ~StepExecutor();

  typedef quadruped_model::LimbEnum LimbEnum;

  /*!
   * Advance in time
   * @param dt the time step to advance [s].
   * @return true if successful, false otherwise.
   */
  bool advance(double dt);

  StepQueue& getQueue();

  void initializeBasePose(const Pose& pose);
  const Pose& getBasePose();

  void initializeJointPositions(LimbEnum limb, const JointPositions& jointPositions);
  bool isJointPositionsAvailable(LimbEnum limb) const;
  const JointPositions& getJointPositions(LimbEnum limb) const;

  void initializeJointVelocities(LimbEnum limb, const JointVelocities& jointVelocities);
  bool isJointVelocitiesAvailable(LimbEnum limb) const;
  const JointVelocities& getJointVelocities(LimbEnum limb) const;

  void initializeJointTorques(LimbEnum limb, const JointTorques& jointTorques);
  bool isJointTorquesAvailable(LimbEnum limb) const;
  const JointTorques& getJointTorques(LimbEnum limb) const;

  void initializeSupportLeg(LimbEnum limb, bool support);
  bool isSupportLeg(LimbEnum limb) const;

 private:
  StepQueue queue_;
  std::shared_ptr<StepCompleter> completer_;
  Pose basePose_;

  std::unordered_map<quadruped_model::LimbEnum, JointPositions, robotUtils::EnumClassHash> jointPositions_;
  std::unordered_map<quadruped_model::LimbEnum, JointVelocities, robotUtils::EnumClassHash> jointVelocities_;
  std::unordered_map<quadruped_model::LimbEnum, JointTorques, robotUtils::EnumClassHash> jointTorques_;
  std::unordered_map<quadruped_model::LimbEnum, bool, robotUtils::EnumClassHash> isSupportLegs_;
};

} /* namespace free_gait */
