/*
 * JointTrajectories.cpp
 *
 *  Created on: March 9, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco_joint_trajectories/JointTrajectories.hpp"
#include "loco/common/StepCompleter.hpp"

// Signal logger
#include "signal_logger/logger.hpp"

// Loco
#include "loco/common/LegLinkGroup.hpp"
#include "loco/common/Step.hpp"
#include "loco/common/TypeDefs.hpp"
#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"
#include "loco/gait_pattern/GaitPatternJointTrajectories.hpp"
#include "loco/limb_coordinator/LimbCoordinatorDynamicGait.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyJointTrajectories.hpp"
#include "loco/torso_control/TorsoControlFreeGait.hpp"
#include "loco/motion_control/VirtualModelController.hpp"
#include "loco/contact_force_distribution/ContactForceDistribution.hpp"
#include "loco/contact_detection/ContactDetectorFeedThrough.hpp"
#include "loco/terrain_perception/TerrainPerceptionFreePlane.hpp"

inline bool fileExists(const std::string pathToFile)
{
  return std::ifstream(pathToFile);
}

namespace loco_joint_trajectories {

JointTrajectories::JointTrajectories()
    : JointTrajectories("JointTrajectories")
{

}

JointTrajectories::JointTrajectories(const std::string& controllerName)
    : Base(controllerName),
      gaitNameString_("JointTrajectories"),
      didSetControllerPath_(false)
{
}

JointTrajectories::~JointTrajectories()
{
}

bool JointTrajectories::create(double dt)
{
  if (!didSetControllerPath_) {
    ROCO_FATAL("[JointTrajectories::create] ERROR: did not set parameter path. Try to call setControllerPath(path) first.");
  }

  // Load parameters.
  std::string taskName { "LocoJointTrajectoriesTask" };
  std::string taskParameterFile { pathToParameterFiles_ + "/" + taskName + ".xml" };

  if (!fileExists(taskParameterFile)) {
    std::cout << "[LocoJointTrajectoriesTask/Constructor] " << "ERROR: "
        << "could not find task parameter file: " << taskParameterFile << std::endl;
    throw std::runtime_error("File does not exist.");
  } else {
    std::cout << "[LocoJointTrajectoriesTask/Constructor] " << "Found task parameter file: "
        << taskParameterFile << std::endl;
  }

  loco::ParameterSet* taskParameterSet = new loco::ParameterSet();
  if (!taskParameterSet->loadXmlDocument(taskParameterFile)) {
    std::cout << "[LocoJointTrajectoriesTask/Constructor] " << "Could not load task parameter file: "
        << taskParameterFile << std::endl;
    delete taskParameterSet;
    throw std::runtime_error("Error FINDING LocoJointTrajectories parameter file.");
  }
  if (!loadTaskParameters(taskParameterSet->getHandle().FirstChild("TaskParameters"))) {
    std::cout << "[LocoJointTrajectoriesTask/Constructor] "
        << "There were errors while loading the task parameter file." << std::endl;
    delete taskParameterSet;
    throw std::runtime_error("Error LOADING LocoJointTrajectories parameter file.");
  }
  delete taskParameterSet;

  std::string gait = gaitNameString_;
  std::cout << "[LocoJointTrajectoriesTask/add] " << "Using Gait: " << gait << std::endl;
  if (!isRealRobot()) gait += "Sim";
  std::string parameterFile = pathToParameterFiles_ + "/" + gait + ".xml";

  if (!fileExists(parameterFile)) {
    std::cout << "[LocoJointTrajectoriesTask/create] " << "ERROR: " << "could not find gait parameter file: "
        << parameterFile << std::endl;
    throw std::runtime_error("File does not exist.");
  }

  setParameterFile(parameterFile);
  std::cout << "[LocoJointTrajectoriesTask/add] " << "Using gait parameter file: " << parameterFile
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
  stepQueue_.reset(new loco::StepQueue());
  stepCompleter_.reset(new loco::StepCompleter());
  terrainPerception_.reset(new loco::TerrainPerceptionFreePlane((loco::TerrainModelFreePlane*) terrainModel_.get(), legs_.get(), torso_.get()));
  contactDetector_.reset(new loco::ContactDetectorFeedThrough());
  gaitPattern_.reset(new loco::GaitPatternJointTrajectories(legs_, torso_, stepQueue_));
  limbCoordinator_.reset(new loco::LimbCoordinatorDynamicGait(legs_.get(), torso_.get(), gaitPattern_.get()));
  torsoController_.reset(new loco::TorsoControlFreeGait(legs_.get(), torso_.get(), terrainModel_.get(), stepQueue_));
  footPlacementStrategy_.reset(new loco::FootPlacementStrategyJointTrajectories(legs_, torso_, stepQueue_));
  contactForceDistribution_.reset(new loco::ContactForceDistribution(torso_, legs_, terrainModel_));
  virtualModelController_.reset(new loco::VirtualModelController(legs_, torso_, contactForceDistribution_));
  locomotionController_.reset(new loco::LocomotionControllerDynamicGait(
      legs_.get(), torso_.get(), terrainPerception_.get(), contactDetector_.get(),
      limbCoordinator_.get(), footPlacementStrategy_.get(), torsoController_.get(),
      virtualModelController_.get(), contactForceDistribution_.get(),
      parameterSet_.get(), gaitPattern_.get(), terrainModel_.get()));

  // Add Variables to logger.
  virtualModelController_.get()->addVariablesToLog(true);
  static_cast<loco::TerrainModelFreePlane*>(terrainModel_.get())->addVariablesToLog(true);
  for (auto leg : *legs_.get()) {
    leg->addVariablesToLog(true);
  }

  signal_logger::logger->updateLogger(true);

  return true;
}

bool JointTrajectories::loadTaskParameters(const TiXmlHandle& handle)
{
  return true;
}

bool JointTrajectories::initialize(double dt)
{
  if (!locomotionController_->initialize(dt)) {
    std::cout << "[LocoJointTrajectoriesTask/init] " << "Could not initialize locomotion controller!" << std::endl;
    return false;
  }
  timeAtInit_ = getTime();
  return true;
}

bool JointTrajectories::advance(double dt)
{
  if (!locomotionController_->advanceMeasurements(dt)) return false;
  if (!stepQueue_->advance(dt)) return false; // https://bitbucket.org/ethz-asl-lr/loco/issue/4/idea-more-flexible-module-handling
  if (!locomotionController_->advanceSetPoints(dt)) return false;

  /* copy desired commands from locomotion controller to robot model */
  for (auto leg : *legs_) {
    for (int i = 0; i < leg->getDesiredJointControlModes().Dimension; i++) {
      getCommand().getBranchNodeActuatorCommands()[leg->getId()][i]->setMode(
          leg->getDesiredJointControlModes()(i));
      getCommand().getBranchNodeActuatorCommands()[leg->getId()][i]->setJointPosition(
          leg->getDesiredJointPositions()(i));
      getCommand().getBranchNodeActuatorCommands()[leg->getId()][i]->setTorque(
          leg->getDesiredJointTorques()(i));
    }
  }

  return true;
}


bool JointTrajectories::change()
{
  return false;
}

bool JointTrajectories::cleanup()
{
  return true;
}

bool JointTrajectories::reset(double dt)
{
  return initialize(dt);
}

void JointTrajectories::setParameterFile(const std::string& parameterFile)
{
  std::cout << "Setting parameter file: " << parameterFile << std::endl;
  parameterFile_ = parameterFile;
}

void JointTrajectories::setControllerPath(const std::string& controllerPath)
{
  didSetControllerPath_ = true;
  pathToParameterFiles_ = controllerPath + "/parameters";
}

} // namespace loco_joint_trajectories
