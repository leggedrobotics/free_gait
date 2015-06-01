/*
 * JointTrajectories.hpp
 *
 *  Created on: Mar 9, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// starlethModel
#include <robot_model/State.hpp>
#include <robot_model/Command.hpp>

// roco
#include <roco/time/TimeStd.hpp>
#include <roco/controllers/Controller.hpp>

// loco
#include "loco/common/LegStarlETH.hpp"
#include "loco/common/TorsoStarlETH.hpp"
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/ParameterSet.hpp"
#include "loco/common/StepQueue.hpp"
#include "loco/common/StepCompleter.hpp"
#include "loco/locomotion_controller/LocomotionControllerBase.hpp"
#include "loco/gait_pattern/GaitPatternBase.hpp"
#include "loco/limb_coordinator/LimbCoordinatorBase.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"
#include "loco/torso_control/TorsoControlBase.hpp"
#include "loco/motion_control/MotionControllerBase.hpp"
#include "loco/contact_force_distribution/ContactForceDistributionBase.hpp"
#include "loco/contact_detection/ContactDetectorBase.hpp"
#include "loco/terrain_perception/TerrainPerceptionBase.hpp"


namespace loco_joint_trajectories {

class JointTrajectories : public roco::controllers::Controller<robot_model::State, robot_model::Command>
{
 public:
  typedef roco::controllers::Controller<robot_model::State, robot_model::Command> Base;

  JointTrajectories();
  JointTrajectories(const std::string& controllerName);
  virtual ~JointTrajectories();

  //! roco implementation.
  virtual bool create(double dt);
  virtual bool initialize(double dt);
  virtual bool advance(double dt);
  virtual bool reset(double dt);
  virtual bool cleanup();
  virtual bool change();
  roco::time::TimeStd timeAtInit_;

  virtual void setControllerPath(const std::string& controllerPath);
  virtual bool loadTaskParameters(const TiXmlHandle& handle);
  void setParameterFile(const std::string& parameterFile);

 private:
  std::string pathToParameterFiles_;
  std::string parameterFile_;
  std::string gaitNameString_;
  bool didSetControllerPath_;

 public:
  //! Legs.
  std::shared_ptr<loco::LegGroup> legs_;
  std::shared_ptr<loco::LegStarlETH> leftForeLeg_;
  std::shared_ptr<loco::LegStarlETH> rightForeLeg_;
  std::shared_ptr<loco::LegStarlETH> leftHindLeg_;
  std::shared_ptr<loco::LegStarlETH> rightHindLeg_;

  //! Base.
  std::shared_ptr<loco::TorsoStarlETH> torso_;
  std::shared_ptr<loco::TorsoControlBase> torsoController_;

  //! Step queue.
  std::shared_ptr<loco::StepQueue> stepQueue_;
  std::shared_ptr<loco::StepCompleter> stepCompleter_;

  //! Terrain.
  std::shared_ptr<loco::TerrainModelBase> terrainModel_;
  std::shared_ptr<loco::TerrainPerceptionBase> terrainPerception_;

  //! Gaits.
  std::shared_ptr<loco::GaitPatternBase> gaitPattern_;
  std::shared_ptr<loco::LimbCoordinatorBase> limbCoordinator_;
  std::shared_ptr<loco::FootPlacementStrategyBase> footPlacementStrategy_;

  //! Strategy.
  std::shared_ptr<loco::ContactForceDistributionBase> contactForceDistribution_;
  std::shared_ptr<loco::MotionControllerBase> virtualModelController_;
  std::shared_ptr<loco::LocomotionControllerBase> locomotionController_;
  std::shared_ptr<loco::ParameterSet> parameterSet_;
  std::shared_ptr<loco::ContactDetectorBase> contactDetector_;
};

} // namespace
