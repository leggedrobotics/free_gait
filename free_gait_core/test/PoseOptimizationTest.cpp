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

// kindr
#include "kindr/common/gtest_eigen.hpp"

using namespace free_gait;
using namespace kindr::common::eigen;

TEST(quadruped, symmetric)
{
  PoseOptimization optimization;

  optimization.setDesiredLegConfiguration( {
    Position(1.0, 0.5, 0.0),
    Position(1.0, -0.5, 0.0),
    Position(-1.0, -0.5, 0.0),
    Position(-1.0, 0.5, 0.0) });

  optimization.setFeetPositions({
    Position(1.0, 0.5, 0.0),
    Position(1.0, -0.5, 0.0),
    Position(-1.0, -0.5, 0.0),
    Position(-1.0, 0.5, 0.0) });

  Pose result;
  EXPECT_TRUE(optimization.compute(result));

  assertEqual(Pose().getTransformationMatrix(), result.getTransformationMatrix(), KINDR_SOURCE_FILE_POS);
}

TEST(quadruped, symmetricWithOffset)
{
  PoseOptimization optimization;

  optimization.setDesiredLegConfiguration( {
    Position(1.0, 0.5, 0.0),
    Position(1.0, -0.5, 0.0),
    Position(-1.0, -0.5, 0.0),
    Position(-1.0, 0.5, 0.0) });

  optimization.setFeetPositions({
    Position(31.0, 20.5, 10.0),
    Position(31.0, 19.5, 10.0),
    Position(29.0, 19.5, 10.0),
    Position(29.0, 20.5, 10.0) });

  Pose startPose;
  startPose.getPosition() << 30.0, 20.0, 10.0;
  optimization.setStartPose(startPose);

  Pose result;
  EXPECT_TRUE(optimization.compute(result));

  assertEqual(startPose.getTransformationMatrix(), result.getTransformationMatrix(), KINDR_SOURCE_FILE_POS);
}

TEST(quadruped, asymmetric)
{
  PoseOptimization optimization;

  optimization.setDesiredLegConfiguration( {
    Position(1.0, 0.5, 0.0),
    Position(1.0, -0.5, 0.0),
    Position(-1.0, -0.5, 0.0),
    Position(-1.0, 0.5, 0.0) });

  optimization.setFeetPositions({
    Position(2.0, 0.5, 0.0),
    Position(1.0, -0.5, 0.0),
    Position(-1.0, -0.5, 0.0),
    Position(-1.0, 0.5, 0.0) });

  Pose result;
  EXPECT_TRUE(optimization.compute(result));
  
  Eigen::Matrix4d expected;
  expected <<  0.9950, 0.0998, 0.0,  0.2500,
              -0.0998, 0.9950, 0.0, -0.0000,
               0.0,    0.0,    1.0,  0.0,
               0.0,    0.0,    0.0,  1.0;

  assertNear(expected, result.getTransformationMatrix(), 1e-3, KINDR_SOURCE_FILE_POS);
}

TEST(quadruped, withYawRotation)
{
  PoseOptimization optimization;

  optimization.setDesiredLegConfiguration( {
    Position(1.0, 0.5, 0.0),
    Position(1.0, -0.5, 0.0),
    Position(-1.0, -0.5, 0.0),
    Position(-1.0, 0.5, 0.0) });

  std::vector<Position> feetPositions;
  kindr::rotations::eigen_impl::EulerAnglesXyzPD rotation(0.0, 0.0, 0.5);
  feetPositions.push_back(rotation.rotate(Position(2.0, 0.5, 0.0)));
  feetPositions.push_back(rotation.rotate(Position(1.0, -0.5, 0.0)));
  feetPositions.push_back(rotation.rotate(Position(-1.0, -0.5, 0.0)));
  feetPositions.push_back(rotation.rotate(Position(-1.0, 0.5, 0.0)));
    optimization.setFeetPositions(feetPositions);

  Pose startPose;
  startPose.getRotation() = rotation;
  optimization.setStartPose(startPose);

  Pose result;
  EXPECT_TRUE(optimization.compute(result));

  Eigen::Matrix4d expected;
  expected <<  0.9950, 0.0998, 0.0,  0.2194, // TODO wrong rotation.
              -0.0998, 0.9950, 0.0,  0.1199,
               0.0,    0.0,    1.0,  0.0,
               0.0,    0.0,    0.0,  1.0;

  assertNear(expected, result.getTransformationMatrix(), 1e-3, KINDR_SOURCE_FILE_POS);
}
