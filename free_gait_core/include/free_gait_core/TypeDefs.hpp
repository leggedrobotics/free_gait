/*
 * TypeDefs.hpp
 *
 *  Created on: Jun 1, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// quadruped model
#include <quadruped_model/common/typedefs.hpp>
#include <quadruped_model/QuadrupedModel.hpp>

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
using QD = quadruped_model::QuadrupedModel::QuadrupedDescription;

using LimbEnum = QD::LimbEnum;
using BranchEnum = QD::BranchEnum;
using JointNodeEnum = QD::JointNodeEnum;
using ContactEnum = QD::ContactEnum;
using FrameTransformEnum = QD::ConcreteTopology::FrameTransformEnum;

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
using JointPositionsLeg = quadruped_model::JointPositionsLimb;
using quadruped_model::JointVelocities;
using JointVelocitiesLeg = quadruped_model::JointVelocitiesLimb;
using quadruped_model::JointAccelerations;
using JointAccelerationsLeg = quadruped_model::JointAccelerationsLimb;
using JointEfforts = quadruped_model::JointTorques;
using JointEffortsLeg = quadruped_model::JointTorquesLimb;

enum class ControlLevel
{
  Position,
  Velocity,
  Acceleration,
  Effort
};

enum class ImpedanceControl
{
  Position,
  Velocity,
  Force
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
