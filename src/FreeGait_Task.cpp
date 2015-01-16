/*
 * FreeGait_Task.cpp
 *
 *  Created on: Jan 13, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "FreeGait_Task.hpp"

// robotUtils
#include "robotUtils/loggers/logger.hpp"

// Loco
#include "loco/common/LegLinkGroup.hpp"
#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"
#include "loco/gait_pattern/GaitPatternFreeGait.hpp"
#include "loco/limb_coordinator/LimbCoordinatorDynamicGait.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyFreeGait.hpp"
#include "loco/torso_control/TorsoControlFreeGait.hpp"
#include "loco/motion_control/VirtualModelController.hpp"
#include "loco/contact_force_distribution/ContactForceDistribution.hpp"
#include "loco/contact_detection/ContactDetectorFeedThrough.hpp"
#include "loco/terrain_perception/TerrainPerceptionFreePlane.hpp"

inline bool fileExists(const std::string pathToFile)
{
  return std::ifstream(pathToFile);
}

namespace robotTask {

FreeGait::FreeGait()
    : Base("FreeGait"),
      gaitNameString_("FreeGait")
{

  // Load parameters.
  std::string taskName { "LocoFreeGaitTask" };
  std::string taskParameterFile { std::string(getenv("LAB_ROOT"))
      + "/starlethTaskLocoFreeGait/parameters/" + taskName + ".xml" };

  if (!fileExists(taskParameterFile)) {
    std::cout << "[LocoFreeGaitTask/Constructor] " << "ERROR: "
        << "could not find task parameter file: " << taskParameterFile << std::endl;
    throw std::runtime_error("File does not exist.");
  } else {
    std::cout << "[LocoFreeGaitTask/Constructor] " << "Found task parameter file: "
        << taskParameterFile << std::endl;
  }

  loco::ParameterSet* taskParameterSet = new loco::ParameterSet();
  if (!taskParameterSet->loadXmlDocument(taskParameterFile)) {
    std::cout << "[LocoFreeGaitTask/Constructor] " << "Could not load task parameter file: "
        << taskParameterFile << std::endl;
    delete taskParameterSet;
    throw std::runtime_error("Error FINDING LocoFreeGait parameter file.");
  }
  if (!loadTaskParameters(taskParameterSet->getHandle().FirstChild("TaskParameters"))) {
    std::cout << "[LocoFreeGaitTask/Constructor] "
        << "There were errors while loading the task parameter file." << std::endl;
    delete taskParameterSet;
    throw std::runtime_error("Error LOADING LocoFreeGait parameter file.");
  }
  delete taskParameterSet;
}

FreeGait::~FreeGait()
{
}

bool FreeGait::create(double dt)
{
  std::string gait = gaitNameString_;
  std::cout << "[LocoFreeGaitTask/add] " << "Using Gait: " << gait << std::endl;
  if (!isRealRobot()) gait += "Sim";
  std::string parameterFile = std::string(getenv("LAB_ROOT")) + "/starlethTaskLocoFreeGait/parameters/" + gait + ".xml";

  if (!fileExists(parameterFile)) {
    std::cout << "[LocoFreeGaitTask/create] " << "ERROR: " << "could not find gait parameter file: "
        << parameterFile << std::endl;
    throw std::runtime_error("File does not exist.");
  }

  setParameterFile(parameterFile);
  std::cout << "[LocoFreeGaitTask/add] " << "Using gait parameter file: " << parameterFile
      << std::endl;

  parameterSet_.reset(new loco::ParameterSet());
  if (!parameterSet_->loadXmlDocument(parameterFile_)) {
    std::cout << "Could not load parameter file: " << parameterFile_ << std::endl;
    return false;
  } else {
    std::cout << "Loaded file: " << parameterFile_ << std::endl;
  }

  terrainModel_.reset(new loco::TerrainModelFreePlane);
  leftForeLeg_.reset (new loco::LegStarlETH("leftFore",  0, getState().getRobotModelPtr()));
  rightForeLeg_.reset(new loco::LegStarlETH("rightFore", 1, getState().getRobotModelPtr()));
  leftHindLeg_.reset (new loco::LegStarlETH("leftHind",  2, getState().getRobotModelPtr()));
  rightHindLeg_.reset(new loco::LegStarlETH("rightHind", 3, getState().getRobotModelPtr()));
  legs_.reset( new loco::LegGroup(leftForeLeg_.get(), rightForeLeg_.get(), leftHindLeg_.get(), rightHindLeg_.get()));
  torso_.reset(new loco::TorsoStarlETH(getState().getRobotModelPtr()));
  terrainPerception_.reset(new loco::TerrainPerceptionFreePlane((loco::TerrainModelFreePlane*) terrainModel_.get(), legs_.get(), torso_.get()));
  contactDetector_.reset(new loco::ContactDetectorFeedThrough());
  gaitPattern_.reset(new loco::GaitPatternFreeGait(legs_.get(), torso_.get()));
  limbCoordinator_.reset(new loco::LimbCoordinatorDynamicGait(legs_.get(), torso_.get(), gaitPattern_.get()));
  torsoController_.reset(new loco::TorsoControlFreeGait(legs_.get(), torso_.get(), terrainModel_.get()));
  footPlacementStrategy_.reset(new loco::FootPlacementStrategyFreeGait(legs_.get(), torso_.get(), terrainModel_.get()));
  contactForceDistribution_.reset(new loco::ContactForceDistribution(torso_, legs_, terrainModel_));
  virtualModelController_.reset(new loco::VirtualModelController(legs_, torso_, contactForceDistribution_));
  locomotionController_.reset(new loco::LocomotionControllerDynamicGait(
      legs_.get(), torso_.get(), terrainPerception_.get(), contactDetector_.get(),
      limbCoordinator_.get(), footPlacementStrategy_.get(), torsoController_.get(),
      virtualModelController_.get(), contactForceDistribution_.get(),
      parameterSet_.get(), gaitPattern_.get(), terrainModel_.get()));

  return true;
}

bool FreeGait::loadTaskParameters(const TiXmlHandle& handle)
{
  return true;
}


bool FreeGait::initialize(double dt)
{
  if (!locomotionController_->initialize(dt)) {
    std::cout << "[LocoFreeGaitTask/init] " << "Could not initialize locomotion controller!" << std::endl;
    return false;
  }
  timeAtInit_ = getTime();
  return true;
}

bool FreeGait::advance(double dt)
{
  if (!locomotionController_->advanceMeasurements(dt)) return false;
  if (!locomotionController_->advanceSetPoints(dt)) return false;

  // Copy desired commands from locomotion controller to robot model.
  robotModel::VectorActMLeg legMode;
  for (auto leg : *legs_) {
    getState().getRobotModelPtr()->act().setPosOfLeg(
        (Eigen::Vector3d) leg->getDesiredJointPositions(), leg->getId());
    getState().getRobotModelPtr()->act().setTauOfLeg(
        (Eigen::Vector3d) leg->getDesiredJointTorques(), leg->getId());
    robotModel::VectorActMLeg modes = leg->getDesiredJointControlModes().matrix();
    getState().getRobotModelPtr()->act().setModeOfLeg(modes, leg->getId());
  }
  return true;
}


bool FreeGait::change() {
  return false;
}

bool FreeGait::cleanup() {
  return true;
}

bool FreeGait::reset(double dt) {
  return initialize(dt);
}

void FreeGait::setParameterFile(const std::string& parameterFile) {
  std::cout << "Setting parameter file: " << parameterFile << std::endl;
  parameterFile_ = parameterFile;
}

} // namespace robotTask
