/*
 * FreeGait_Task.cpp
 *
 *  Created on: Jan 13, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "FreeGait_Task.hpp"


//#include "robotUtils/loggers/logger.hpp"

//#include <fstream>

/*******************
 * Terrain classes *
 *******************/
#include "loco/common/TerrainModelFreePlane.hpp"
#include "loco/common/TerrainModelHorizontalPlane.hpp"
#include "loco/terrain_perception/TerrainPerceptionFreePlane.hpp"
#include "loco/terrain_perception/TerrainPerceptionHorizontalPlane.hpp"
/*******************/


#include "loco/common/LegLinkGroup.hpp"
#include "loco/contact_detection/ContactDetectorConstantDuringStance.hpp"
#include "loco/contact_detection/ContactDetectorFeedThrough.hpp"


/***********************************
 * Foot placement strategy classes *
 ***********************************/
#include "loco/foot_placement_strategy/FootPlacementStrategyFreeGait.hpp"
/***********************************/


/****************************
 * Torso controller classes *
 ****************************/
#include "loco/torso_control/TorsoControlFreeGait.hpp"
/****************************/


/******************************
 * Mission controller classes *
 ******************************/
#include "loco/mission_control/MissionControlJoystick.hpp"
#include "loco/mission_control/MissionControlStaticGait.hpp"
/******************************/


inline bool fileExists( const std::string pathToFile ) {
  return std::ifstream(pathToFile);
}


namespace robotTask {

FreeGait::FreeGait()
    : Base("FreeGait"),
      gaitNameString_("FreeGait")
{

  /************************
   * Load task parameters *
   ************************/
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


FreeGait::~FreeGait() {
}

bool FreeGait::create(double dt) {
  /******************************
   * Choose gait parameter file *
   ******************************/
  std::string gait = gaitNameString_;

  std::cout << "[LocoFreeGaitTask/add] " << "Using Gait: " << gait << std::endl;

  if (!isRealRobot()) {
      gait += "Sim";
  }

  std::string parameterFile = std::string(getenv("LAB_ROOT")) + "/starlethTaskLocoFreeGait/parameters/" + gait + ".xml";

  if (!fileExists(parameterFile)) {
    std::cout << "[LocoFreeGaitTask/create] " << "ERROR: " << "could not find gait parameter file: "
        << parameterFile << std::endl;
    throw std::runtime_error("File does not exist.");
  } else {
  }

  setParameterFile(parameterFile);
  std::cout << "[LocoFreeGaitTask/add] " << "Using gait parameter file: " << parameterFile
      << std::endl;
  /******************************/

  parameterSet_.reset(new loco::ParameterSet());
  if (!parameterSet_->loadXmlDocument(parameterFile_)) {
    std::cout << "Could not load parameter file: " << parameterFile_  << std::endl;
    return false;
  }
  else {
    std::cout << "Loaded file: " << parameterFile_  << std::endl;
  }

  /* create terrain model */
//  if (useHorizontalPlane_) {
//    terrainModel_.reset(new loco::TerrainModelHorizontalPlane);
//  } else {
    terrainModel_.reset(new loco::TerrainModelFreePlane);
//  }

  /* create legs and group of legs */
  leftForeLeg_.reset (new loco::LegStarlETH("leftFore",  0, getState().getRobotModelPtr()));
  rightForeLeg_.reset(new loco::LegStarlETH("rightFore", 1, getState().getRobotModelPtr()));
  leftHindLeg_.reset (new loco::LegStarlETH("leftHind",  2, getState().getRobotModelPtr()));
  rightHindLeg_.reset(new loco::LegStarlETH("rightHind", 3, getState().getRobotModelPtr()));
  legs_.reset( new loco::LegGroup(leftForeLeg_.get(), rightForeLeg_.get(), leftHindLeg_.get(), rightHindLeg_.get()));

  /* create torso */
  torso_.reset(new loco::TorsoStarlETH(getState().getRobotModelPtr()));

  /* create terrain perception */
//  if (useHorizontalPlane_) {
//    std::cout << "[LocoFreeGaitTask/add] " << "Using horizontal plane" << std::endl;
//    terrainPerception_.reset(
//        new loco::TerrainPerceptionHorizontalPlane(
//            (loco::TerrainModelHorizontalPlane*) terrainModel_.get(),
//            legs_.get(), torso_.get()));
//  } else {
    std::cout << "[LocoFreeGaitTask/add] " << "Using free plane" << std::endl;
    terrainPerception_.reset(
        new loco::TerrainPerceptionFreePlane(
            (loco::TerrainModelFreePlane*) terrainModel_.get(), legs_.get(),
            torso_.get()));
//  }

  /* create locomotion controller */
  contactDetector_.reset(new loco::ContactDetectorConstantDuringStance(legs_.get()));
//  contactDetector_.reset(new loco::ContactDetectorFeedThrough());
//  gaitPatternAPS_.reset(new loco::GaitPatternAPS);
  gaitPattern_.reset(new loco::GaitPatternFreeGait());
  limbCoordinator_.reset(new loco::LimbCoordinatorDynamicGait(legs_.get(), torso_.get(), gaitPattern_.get()));


  //--- FOOT PLACEMENT AND TORSO CONTROLLER
    std::cout << "[LocoFreeGaitTask/add] "
              << "Using static gait controller and fps" << std::endl;
//    std::cout << "teststest: " << torso_->getMeasuredState().getOrientationWorldToControl() << std::endl;
    torsoController_.reset(new loco::TorsoControlFreeGait(legs_.get(), torso_.get(), terrainModel_.get()));
    footPlacementStrategy_.reset(new loco::FootPlacementStrategyFreeGait(legs_.get(), torso_.get(), terrainModel_.get()));

  contactForceDistribution_.reset(new loco::ContactForceDistribution(torso_, legs_, terrainModel_));
  virtualModelController_.reset(new loco::VirtualModelController(legs_, torso_, contactForceDistribution_));
  locomotionController_.reset(new loco::LocomotionControllerDynamicGait(legs_.get(),
                                                                        torso_.get(),
                                                                        terrainPerception_.get(),
                                                                        contactDetector_.get(),
                                                                        limbCoordinator_.get(),
                                                                        footPlacementStrategy_.get(),
                                                                        torsoController_.get(),
                                                                        virtualModelController_.get(),
                                                                        contactForceDistribution_.get(),
                                                                        parameterSet_.get(),
                                                                        gaitPattern_.get(),
                                                                        terrainModel_.get()));

  return true;
} // add


bool FreeGait::loadTaskParameters(const TiXmlHandle& handle) {
//  TiXmlElement* pElem;
//  TiXmlHandle hFPS(handle.FirstChild("Modules"));
//
//  pElem = hFPS.Element();
//
//  if (!pElem) {
//    std::cout << db_helper::magenta << "[LocoFreeGaitTask/loadTaskParameters] "
//              << db_helper::red
//              << "ERROR: Could not find TaskParameters:Modules"
//              << db_helper::def << std::endl;
//    return false;
//  }
//
//  pElem = hFPS.FirstChild("StaticGait").Element();
//  if (!pElem) {
//    std::cout << db_helper::magenta << "[LocoFreeGaitTask/loadTaskParameters] "
//              << db_helper::red
//              << "ERROR: Could not find TaskParameters:Modules:StaticGait"
//              << db_helper::def << std::endl;
//    return false;
//  }
//
//  std::string startWithStand;
//  if( pElem->QueryStringAttribute("startWithStand", &startWithStand) != TIXML_SUCCESS ) {
//    std::cout << db_helper::magenta << "[LocoCrawlingTask/loadTaskParameters] "
//              << db_helper::red
//              << "ERROR: Could not find attribute 'startWithStand' in TaskParameters:Modules:StaticGait"
//              << db_helper::def << std::endl;
//    return false;
//  }
//  startInStandConfiguration_ = !startWithStand.compare("yes") ? true : false;
//
//
//  pElem = hFPS.FirstChild("FreePlane").Element();
//  if (!pElem) {
//    std::cout << db_helper::magenta << "[LocoCrawlingTask/loadTaskParameters] "
//              << db_helper::red
//              << "ERROR: Could not find TaskParameters:Modules:FreePlane"
//              << db_helper::def << std::endl;
//    return false;
//  }
//  std::string useFreePlane;
//  if( pElem->QueryStringAttribute("useFreePlane", &useFreePlane) != TIXML_SUCCESS ) {
//    std::cout << db_helper::magenta << "[LocoCrawlingTask/loadParameters] "
//              << db_helper::red
//              << "ERROR: Could not find attribute 'useFreePlane' in TaskParameters:Modules:FreePlane"
//              << db_helper::def << std::endl;
//    printf("Could not find initial parameter set!\n");
//    return false;
//  }
//  useHorizontalPlane_ = !useFreePlane.compare("yes") ? false : true;

  return true;
}


bool FreeGait::initialize(double dt) {

    // Set pointer to static CoM control in foot placement strategy
//  loco::FootPlacementStrategyFreeGait* footPlacementStrategy =
//      static_cast<loco::FootPlacementStrategyFreeGait*>(footPlacementStrategy_.get());
//  loco::TorsoControlStaticGait* torsoControl =
//      static_cast<loco::TorsoControlStaticGait*>(torsoController_.get());
//  footPlacementStrategy->setCoMControl(torsoControl->getCoMControl());

    for (auto leg:*legs_.get()) {
      leg->setIsInStandConfiguration(true);
    }


  if (!locomotionController_->initialize(dt)) {
    std::cout << "[LocoFreeGaitTask/init] " << "Could not initialize locomotion controller!" << std::endl;
    return false;
  }


//  if (useStaticGait_) {
//    if (startInStandConfiguration_) {
//      std::cout << db_helper::magenta << "[LocoCrawlingTask/init] "
//                << db_helper::blue << "Starting in stand configuration."
//                << db_helper::def << std::endl;
//    }
//    else {
//      std::cout << db_helper::magenta << "[LocoCrawlingTask/init] "
//                << db_helper::blue << "Starting in walk configuration."
//                << db_helper::def << std::endl;
//      for (auto leg: *legs_.get()) {
//        leg->setIsInStandConfiguration(false);
//      }
//      loco::TorsoControlStaticGait& torsoController = static_cast<loco::TorsoControlStaticGait&>(locomotionController_->getTorsoController());
//      torsoController.setIsInStandConfiguration(false);
//      locomotionController_->getFootPlacementStrategy()->resumeWalking();
//    }
//  }

  timeAtInit_ = getTime();

  return true;
}


bool FreeGait::advance(double dt)
{


  if(!locomotionController_->advanceMeasurements(dt)) {
   //    activateSafetyPilot(false,false);
  }
//
//  if (!useStaticGait_) {
//    loco::LinearVelocity desLinearVelocityBaseInControlFrame = missionController_->getDesiredBaseTwistInHeadingFrame().getTranslationalVelocity();
//    loco::LocalAngularVelocity desAngularVelocityBaseInControlFrame = missionController_->getDesiredBaseTwistInHeadingFrame().getRotationalVelocity();
//    torso_->getDesiredState().setLinearVelocityBaseInControlFrame(desLinearVelocityBaseInControlFrame);
//    torso_->getDesiredState().setAngularVelocityBaseInControlFrame(desAngularVelocityBaseInControlFrame);
//  }


  /* execute locomotion controller */
  if(!locomotionController_->advanceSetPoints(dt)) {
//    activateSafetyPilot(false,false);
  }

  /* copy desired commands from locomotion controller to robot model */
  robotModel::VectorActMLeg legMode;
  int iLeg = 0;
  for (auto leg : *legs_) {
    getState().getRobotModelPtr()->act().setPosOfLeg((Eigen::Vector3d)leg->getDesiredJointPositions() ,iLeg);
    getState().getRobotModelPtr()->act().setTauOfLeg((Eigen::Vector3d)leg->getDesiredJointTorques() ,iLeg);
    robotModel::VectorActMLeg modes = leg->getDesiredJointControlModes().matrix();
    std::cout << modes << std::endl;
    getState().getRobotModelPtr()->act().setModeOfLeg( modes,iLeg);
    iLeg++;
  }

  return true;
}


bool FreeGait::change() {
  return true;
}

bool FreeGait::cleanup() {
  return true;
}

bool FreeGait::reset(double dt) {
  return initialize(dt);
}


loco::LocomotionControllerDynamicGait* FreeGait::getLocomotionController() {
  return locomotionController_.get();
}


loco::ParameterSet* FreeGait::getParameterSet() {
  return parameterSet_.get();
}


loco::TerrainPerceptionBase* FreeGait::getTerrainPerceptionBase() {
  return terrainPerception_.get();
}


loco::TerrainModelBase* FreeGait::getTerrainModelBase() {
  return terrainModel_.get();
}


loco::TorsoBase* FreeGait::getTorsoBase() {
  return torso_.get();
}


void FreeGait::setParameterFile(const std::string& parameterFile) {
  std::cout << "Setting parameter file: " << parameterFile << std::endl;
  parameterFile_ = parameterFile;
} // set parameter file

} // namespace robotTask

