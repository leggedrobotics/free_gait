/*
 * StepParameters.hpp
 *
 *  Created on: Feb 18, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/step/StepCompleter.hpp"

namespace free_gait {

class StepParameters
{
 public:
  StepParameters() {}
  virtual ~StepParameters() {};

  friend class StepCompleter;

 protected:

  struct FootstepParameters
  {
    std::string profileType = "triangle";
    double profileHeight = 0.05;
    double averageVelocity = 0.1;
    double liftOffVelocity = 0.0;
    double touchdownVelocity = 0;
  } footTargetParameters_;

  struct LegModeParameters
  {
    double duration = 0.0025;
    std::string frameId = "base";
  } legModeParameters_;

  struct BaseAutoParameters
  {
    std::string controllerType = "virtual_model_control";
    double averageLinearVelocity = 0.05;
    double averageAngularVelocity = 0.1;
    double supportMargin = 0.04;
    double minimumDuration_ = 0.2;
    PlanarStance nominalPlanarStanceInBaseFrame;

    BaseAutoParameters()
    {
      Position2 position;
      position << 0.385, 0.25;
      nominalPlanarStanceInBaseFrame.emplace(LimbEnum::LF_LEG, position);
      nominalPlanarStanceInBaseFrame.emplace(LimbEnum::RF_LEG, Position2(Eigen::Vector2d(position(0), -position(1))));
      nominalPlanarStanceInBaseFrame.emplace(LimbEnum::LH_LEG, Position2(Eigen::Vector2d(-position(0), position(1))));
      nominalPlanarStanceInBaseFrame.emplace(LimbEnum::RH_LEG, Position2(Eigen::Vector2d(-position(0), -position(1))));
    }

  } baseAutoParameters_;

  struct BaseTrajectoryParameters
  {
    BaseTrajectoryParameters()
    {
    }

  } baseTrajectoryParameters_;
};

} /* namespace */
