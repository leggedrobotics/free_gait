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


  const ControlSetup& getBaseControlSetup() const;
  const Pose& getBasePose();

  const ControlSetup& getLegControlSetup(LimbEnum limb) const;
  const JointPositions& getJointPositions(LimbEnum limb) const;
  const JointVelocities& getJointVelocities(LimbEnum limb) const;
  const JointEfforts& getJointEfforts(LimbEnum limb) const;

  bool isSupportLeg(LimbEnum limb) const;
  bool isIgnoreContact(LimbEnum limb) const;

 protected:
  void virtual updateWithMeasuredBasePose() = 0;
  void virtual updateWithMeasuredBaseTwist() = 0;
  void virtual updateWithMeasuredJointPositions(LimbEnum limb) = 0;
  void virtual updateWithMeasuredJointVelocities(LimbEnum limb) = 0;
  void virtual updateWithMeasuredJointEfforts(LimbEnum limb) = 0;
  void virtual updateWithActualSupportLeg(LimbEnum limb) = 0;

 private:
  StepQueue queue_;
  std::shared_ptr<StepCompleter> completer_;

  Pose basePose_;
  ControlSetup baseControlSetup_;
  std::unordered_map<quadruped_model::LimbEnum, ControlSetup, robotUtils::EnumClassHash> legControlSetups_; // TODO Maybe summarize this in a leg class.
  std::unordered_map<quadruped_model::LimbEnum, JointPositions, robotUtils::EnumClassHash> jointPositions_;
  std::unordered_map<quadruped_model::LimbEnum, JointVelocities, robotUtils::EnumClassHash> jointVelocities_;
  std::unordered_map<quadruped_model::LimbEnum, JointEfforts, robotUtils::EnumClassHash> jointEfforts_;
  std::unordered_map<quadruped_model::LimbEnum, bool, robotUtils::EnumClassHash> isSupportLegs_;
  std::unordered_map<quadruped_model::LimbEnum, bool, robotUtils::EnumClassHash> ignoreContact_;
};

} /* namespace free_gait */
