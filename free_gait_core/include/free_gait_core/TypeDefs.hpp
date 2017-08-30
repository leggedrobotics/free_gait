/*
 * TypeDefs.hpp
 *
 *  Created on: Jun 1, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// quadruped model
#include <quadruped_model/common/enums.hpp>
#include <quadruped_model/common/typedefs.hpp>

// STL
#include <unordered_map>
#include <map>

namespace free_gait {

struct EnumClassHash
{
  template<typename T>
  std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};

// Import enum aliases.
using LimbEnum = quadruped_model::LimbEnum;
using BranchEnum = quadruped_model::BranchEnum;
using JointNodeEnum = quadruped_model::JointNodeEnum;
using ContactEnum = quadruped_model::ContactEnum;

// Import kindr aliases.
using Transform = romo::Pose;
using romo::Pose;
using romo::Twist;
using romo::RotationQuaternion;
using romo::AngleAxis;
using romo::RotationMatrix;
using romo::EulerAnglesZyx;
using romo::RotationVector;
using romo::EulerAnglesXyz;
using romo::EulerAnglesXyzDiff;
using romo::Position;
using Position2 = kindr::Position<double, 2>;
using romo::LinearVelocity;
using romo::LocalAngularVelocity;
using romo::EulerAnglesZyxDiff;
using romo::LinearAcceleration;
using romo::AngularAcceleration;
using romo::Force;
using romo::Torque;
using romo::Vector;

// Import robot-specific kindr quantities.
using quadruped_model::GeneralizedCoordinates;
using quadruped_model::GeneralizedVelocities;
using quadruped_model::GeneralizedAccelerations;
using quadruped_model::JointPositions;
using quadruped_model::JointPositionsLeg;
using quadruped_model::JointVelocities;
using quadruped_model::JointVelocitiesLeg;
using quadruped_model::JointAccelerations;
using quadruped_model::JointAccelerationsLeg;
typedef quadruped_model::JointTorques JointEfforts;
typedef quadruped_model::JointTorquesLeg JointEffortsLeg;

enum class ControlLevel
{
  Position,
  Velocity,
  Acceleration,
  Effort
};

const std::vector<LimbEnum> limbEnumCounterClockWiseOrder = { LimbEnum::LF_LEG,
                                                              LimbEnum::LH_LEG,
                                                              LimbEnum::RH_LEG,
                                                              LimbEnum::RF_LEG };

typedef std::unordered_map<ControlLevel, bool, EnumClassHash> ControlSetup;
typedef std::unordered_map<LimbEnum, Position, EnumClassHash> Stance;
typedef std::unordered_map<LimbEnum, Position2, EnumClassHash> PlanarStance;

struct CompareByCounterClockwiseOrder;
void getFootholdsCounterClockwiseOrdered(const Stance& stance, std::vector<Position>& footholds);

} // namespace
