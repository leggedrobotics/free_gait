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

// stl
#include <unordered_map>

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
using ContactEnum = quadruped_model::ContactEnum;

// Import kindr aliases.
using quadruped_model::Pose;
using quadruped_model::Twist;
using quadruped_model::RotationQuaternion;
using quadruped_model::AngleAxis;
using quadruped_model::RotationMatrix;
using quadruped_model::EulerAnglesZyx;
using quadruped_model::RotationVector;
using quadruped_model::EulerAnglesXyz;
using quadruped_model::EulerAnglesXyzDiff;
using quadruped_model::Position;
using Position2 = kindr::phys_quant::eigen_impl::Position<double, 2>;
using quadruped_model::LinearVelocity;
using quadruped_model::LocalAngularVelocity;
using quadruped_model::EulerAnglesZyxDiff;
using quadruped_model::LinearAcceleration;
using quadruped_model::AngularAcceleration;
using quadruped_model::Force;
using quadruped_model::Torque;
using quadruped_model::Vector;

// Import robot-specific kindr quantities.
using quadruped_model::GeneralizedCoordinates;
using quadruped_model::GeneralizedVelocities;
using quadruped_model::GeneralizedAccelerations;
//using quadruped_model::JointPositions;
//using quadruped_model::JointVelocities;
//using quadruped_model::JointTorques;

typedef kindr::phys_quant::eigen_impl::Position<double, Eigen::Dynamic> JointPositions;
typedef kindr::phys_quant::eigen_impl::Velocity<double, Eigen::Dynamic> JointVelocities;
typedef kindr::phys_quant::eigen_impl::Acceleration<double, Eigen::Dynamic> JointAccelerations;
typedef kindr::phys_quant::eigen_impl::Torque<double, Eigen::Dynamic> JointEfforts;

enum class ControlLevel
{
  Position,
  Velocity,
  Acceleration,
  Effort
};

typedef std::unordered_map<ControlLevel, bool, EnumClassHash> ControlSetup;
typedef std::unordered_map<LimbEnum, Position, EnumClassHash> Stance;
typedef std::unordered_map<LimbEnum, Position2, EnumClassHash> PlanarStance;

} // namespace
