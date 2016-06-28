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
}
