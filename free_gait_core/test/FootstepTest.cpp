/*
 * FootstepTest.cpp
 *
 *  Created on: Feb 11, 2017
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/leg_motion/Footstep.hpp"

// gtest
#include <gtest/gtest.h>

using namespace free_gait;

TEST(footstep, heightCheck)
{
  Footstep footstep(LimbEnum::LF_LEG);
  Position start(0.0, 0.0, 0.0);
  Position target(0.3, 0.0, 0.3);
  footstep.setStartPosition("map", start);
  footstep.setTargetPosition("map", target);
  double height = 0.05;
  footstep.setProfileHeight(0.05);
  footstep.setProfileType("trapezoid");
  footstep.setAverageVelocity(0.15);
  ASSERT_TRUE(footstep.compute(true));

  for (double time = 0.0; time < footstep.getDuration(); time += 0.001) {
    Position position = footstep.evaluatePosition(time);
    EXPECT_LT(position.z(), target.z() + height + 0.001);
    EXPECT_GT(position.x(), start.x() - 0.001);
    EXPECT_LT(position.x(), target.x() + 0.001);
  }
}
