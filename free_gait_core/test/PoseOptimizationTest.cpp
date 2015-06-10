/*
 * PoseOptimizationTest.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/PoseOptimization.hpp"

// gtest
#include <gtest/gtest.h>

using namespace free_gait;

TEST(quadruped, symmetric)
{
  PoseOptimization optimization;
  optimization.setDesiredLegConfiguration( { Position(1.0, 0.5, 0.0), Position(1.0, -0.5, 0.0),
      Position(-1.0, -0.5, 0.0), Position(-1.0, 0.5, 0.0) });
  optimization.setFeetPositions({ Position(31.0, 20.5, 0.0), Position(31.0, 19.5, 0.0),
      Position(29.0, 19.5, 0.0), Position(29.0, 20.5, 0.0) });
  Pose result;
  EXPECT_TRUE(optimization.compute(result));
  std::cout << result << std::endl;
}
