/*
 * PoseOptimizationTest.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/PoseOptimization.hpp"

// gtest
#include <gtest/gtest.h>

using namespace free_gait;

TEST(quadruped, symmetric)
{
  PoseOptimization optimization;
  grid_map::Polygon test;
  EXPECT_TRUE(1==1);
}
