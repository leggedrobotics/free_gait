/*
 * StepTest.cpp
 *
 *  Created on: Oct 27, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/step/Step.hpp"

// gtest
#include <gtest/gtest.h>

// kindr
#include "kindr/common/gtest_eigen.hpp"

using namespace free_gait;

TEST(step, initialization)
{
  Step step;
  ASSERT_FALSE(step.hasBaseMotion());
  ASSERT_FALSE(step.hasLegMotion());
  ASSERT_FALSE(step.hasLegMotion(LimbEnum::RF_LEG));
  Step stepCopy(step);
  ASSERT_FALSE(step.hasBaseMotion());
  ASSERT_FALSE(step.hasLegMotion());
  ASSERT_FALSE(step.hasLegMotion(LimbEnum::RF_LEG));
//  PoseOptimization optimization;
//
//  optimization.setDesiredLegConfiguration( {
//    Position(1.0, 0.5, 0.0),
//    Position(1.0, -0.5, 0.0),
//    Position(-1.0, -0.5, 0.0),
//    Position(-1.0, 0.5, 0.0) });
//
//  optimization.setFeetPositions({
//    Position(1.0, 0.5, 0.0),
//    Position(1.0, -0.5, 0.0),
//    Position(-1.0, -0.5, 0.0),
//    Position(-1.0, 0.5, 0.0) });
//
//  Pose result;
//  ASSERT_TRUE(optimization.optimize(result));
//
//  assertNear(Pose().getTransformationMatrix(), result.getTransformationMatrix(), 1e-3, KINDR_SOURCE_FILE_POS);
}
