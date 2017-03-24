/*
 * PoseOptimizationSQPTest.cpp
 *
 *  Created on: Mar 22, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationSQP.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationObjectiveFunction.hpp"
#include "free_gait_core/pose_optimization/PoseParameterization.hpp"
#include "AdapterDummy.hpp"

#include <grid_map_core/Polygon.hpp>
#include <gtest/gtest.h>
#include <kindr/Core>
#include <kindr/common/gtest_eigen.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>
#include <numopt_sqp/SQPFunctionMinimizer.hpp>
#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>

using namespace free_gait;

TEST(PoseOptimizationSQP, PoseParameterization)
{
  PoseParameterization result, p;
  p.setRandom(p.getParams());
  numopt_common::Vector dp = numopt_common::Vector::Random(p.getLocalSize());
  result.plus(result.getParams(), p.getParams(), dp);
  Position translation = result.getPosition() - p.getPosition();
  RotationVector rotation(result.getOrientation().boxMinus(p.getOrientation()));
  kindr::assertNear(dp.head(3), translation.vector(), 1e-3, KINDR_SOURCE_FILE_POS);
  kindr::assertNear(dp.tail(3), rotation.vector(), 1e-3, KINDR_SOURCE_FILE_POS);
}

TEST(PoseOptimizationSQP, ObjectiveFunction)
{
  AdapterDummy adapter;
//  State state;
//  state.setPoseBaseToWorld(Pose(Position(0.0, 0.0, 0.1), RotationQuaternion()));
//  state.setRandom();
//  adapter.setInternalDataFromState(state);
  PoseOptimizationObjectiveFunction objective(adapter, adapter.getState());
  Stance nominalStance;
  nominalStance[LimbEnum::LF_LEG] = Position(0.3, 0.2, -0.5);
  nominalStance[LimbEnum::RF_LEG] = Position(0.3, -0.2, -0.5);
  nominalStance[LimbEnum::LH_LEG] = Position(-0.3, 0.2, -0.5);
  nominalStance[LimbEnum::RH_LEG] = Position(-0.3, -0.2, -0.5);
  objective.setNominalStance(nominalStance);
  Stance currentStance;
  currentStance[LimbEnum::LF_LEG] = Position(0.3, 0.2, 0.0);
  currentStance[LimbEnum::RF_LEG] = Position(0.3, -0.2, 0.0);
  currentStance[LimbEnum::LH_LEG] = Position(-0.3, 0.2, 0.0);
  currentStance[LimbEnum::RH_LEG] = Position(-0.3, -0.2, 0.0);
  objective.setStance(currentStance);

  PoseParameterization params;
  params.setIdentity(params.getParams());
  numopt_common::Scalar value;
  objective.computeValue(value, params);
  // Each leg has 0.5 m error.
  EXPECT_EQ(4 * std::pow(0.5, 2), value);

  params.setPose(Pose(Position(0.0, 0.0, 0.1), RotationQuaternion()));
  objective.computeValue(value, params);
  // Each leg has 0.4 m error.
  EXPECT_EQ(4 * std::pow(0.4, 2), value);

  numopt_common::Vector gradient;
  objective.getLocalGradient(gradient, params);
  std::cerr << gradient << std::endl;
}

