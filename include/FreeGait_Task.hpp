/*
 * FreeGait_Task.hpp
 *
 *  Created on: Jan 13, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "robotTask/tasks/TaskRobotBase.hpp"
#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"

//#include "loco/gait_pattern/GaitPatternAPS.hpp"
//#include "loco/gait_pattern/GaitPatternFlightPhases.hpp"
#include "loco/gait_pattern/GaitPatternFreeGait.hpp"
#include "loco/limb_coordinator/LimbCoordinatorDynamicGait.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"
#include "loco/torso_control/TorsoControlBase.hpp"
#include "loco/motion_control/VirtualModelController.hpp"
#include "loco/contact_force_distribution/ContactForceDistribution.hpp"
#include "loco/contact_detection/ContactDetectorBase.hpp"

#include "loco/mission_control/MissionControlBase.hpp"
#include "loco/tools/TuningWithSliderboardLocoDynamicGait.hpp"

#include "loco/common/LegStarlETH.hpp"
#include "loco/common/TorsoStarlETH.hpp"
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/ParameterSet.hpp"
#include <memory>

/****************
 * roco headers *
 ****************/
#include <roco/time/TimeStd.hpp>
#include <roco/controllers/Controller.hpp>
#include <starlethModel/State.hpp>
#include <starlethModel/Command.hpp>
/****************/

namespace robotTask {

class FreeGait: public roco::controllers::Controller<robotModel::State, robotModel::Command> {
  public:
    typedef roco::controllers::Controller<robotModel::State, robotModel::Command> Base;

  public:

    FreeGait();
    virtual ~FreeGait();

    /***********************
     * roco implementation *
     ***********************/
    virtual bool create(double dt);
    virtual bool initialize(double dt);
    virtual bool advance(double dt);
    virtual bool reset(double dt);
    virtual bool cleanup();
    virtual bool change();
    roco::time::TimeStd timeAtInit_;
    /***********************/

    virtual bool loadTaskParameters(const TiXmlHandle& handle);

    loco::LocomotionControllerDynamicGait* getLocomotionController();
    loco::ParameterSet* getParameterSet();
    void setParameterFile(const std::string& parameterFile);

    /********************
     * Accessor methods *
     ********************/
    loco::TerrainPerceptionBase* getTerrainPerceptionBase();
    loco::TerrainModelBase* getTerrainModelBase();
    loco::TorsoBase* getTorsoBase();
    /********************/

  private:
    std::string pathToParameterFiles_;
    std::string parameterFile_;
    std::string gaitNameString_;

  public:
    // legs
    std::shared_ptr<loco::LegGroup> legs_;
    std::shared_ptr<loco::LegStarlETH> leftForeLeg_;
    std::shared_ptr<loco::LegStarlETH> rightForeLeg_;
    std::shared_ptr<loco::LegStarlETH> leftHindLeg_;
    std::shared_ptr<loco::LegStarlETH> rightHindLeg_;

    // body
    std::shared_ptr<loco::TorsoStarlETH> torso_;
    std::shared_ptr<loco::TorsoControlBase> torsoController_;

    // terrain
    std::shared_ptr<loco::TerrainModelBase> terrainModel_;
    std::shared_ptr<loco::TerrainPerceptionBase> terrainPerception_;

    // gaits
//    std::shared_ptr<loco::GaitPatternAPS> gaitPatternAPS_;
    std::shared_ptr<loco::GaitPatternFreeGait> gaitPattern_;
    std::shared_ptr<loco::LimbCoordinatorDynamicGait> limbCoordinator_;
    std::shared_ptr<loco::FootPlacementStrategyBase> footPlacementStrategy_;

    // strategy
    std::shared_ptr<loco::ContactForceDistribution> contactForceDistribution_;
    std::shared_ptr<loco::VirtualModelController> virtualModelController_;
    std::shared_ptr<loco::LocomotionControllerDynamicGait> locomotionController_;
    std::shared_ptr<loco::ParameterSet> parameterSet_;
    std::shared_ptr<loco::ContactDetectorBase> contactDetector_;
};

} // namespace

