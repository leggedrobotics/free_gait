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
    double profileHeight = 0.08;
    double averageVelocity = 0.8;
    double liftOffSpeed = 0.06;
    double touchdownSpeed = 0.04;
    double minimumDuration_ = 0.1; // TODO Debug.
  } footTargetParameters;

  struct EndEffectorTargetParameters
  {
    double averageVelocity = 0.15;
    double minimumDuration_ = 0.1;
  } endEffectorTargetParameters;

  struct LegModeParameters
  {
    double duration = 0.5;
    std::string frameId = "base";
  } legModeParameters;

  struct BaseAutoParameters
  {
    double averageLinearVelocity = 0.18;
    double averageAngularVelocity = 0.32;
    double supportMargin = 0.05;
    double minimumDuration = 0.3;
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

  } baseAutoParameters;

  struct BaseTargetParameters
  {
    double averageLinearVelocity = 0.05;
    double averageAngularVelocity = 0.1;
    double minimumDuration = 0.7;
  } baseTargetParameters;

  struct BaseTrajectoryParameters
  {
    BaseTrajectoryParameters()
    {
    }

  } baseTrajectoryParameters;
};

} /* namespace */
