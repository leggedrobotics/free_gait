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

// Grid map
#include <grid_map_core/Polygon.hpp>

using namespace free_gait;
using namespace kindr::common::eigen;

TEST(quadruped, symmetricUnconstrained)
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

TEST(quadruped, symmetricWithOffsetUnconstrained)
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

TEST(quadruped, asymmetricUnconstrained)
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
  expected <<  0.9950, -0.0998, 0.0, 0.25,
               0.0998,  0.9950, 0.0, 0.0,
               0.0,     0.0,    1.0, 0.0,
               0.0,     0.0,    0.0, 1.0;

  assertNear(expected, result.getTransformationMatrix(), 1e-3, KINDR_SOURCE_FILE_POS);
}

TEST(quadruped, symmetricWithYawRotationUnconstrained)
{
  PoseOptimization optimization;

  optimization.setDesiredLegConfiguration( {
    Position(1.0, 0.5, 0.0),
    Position(1.0, -0.5, 0.0),
    Position(-1.0, -0.5, 0.0),
    Position(-1.0, 0.5, 0.0) });

  std::vector<Position> feetPositions;
  kindr::rotations::eigen_impl::EulerAnglesZyxPD rotation(0.5, 0.0, 0.0);
  feetPositions.push_back(rotation.inverseRotate(Position(1.0, 0.5, 0.0)));
  feetPositions.push_back(rotation.inverseRotate(Position(1.0, -0.5, 0.0)));
  feetPositions.push_back(rotation.inverseRotate(Position(-1.0, -0.5, 0.0)));
  feetPositions.push_back(rotation.inverseRotate(Position(-1.0, 0.5, 0.0)));
  optimization.setFeetPositions(feetPositions);

  Pose startPose;
  startPose.getRotation() = rotation;
  optimization.setStartPose(startPose);

  Pose result;
  EXPECT_TRUE(optimization.compute(result));

  assertNear(startPose.getTransformationMatrix(), result.getTransformationMatrix(), 1e-3, KINDR_SOURCE_FILE_POS);
}

TEST(quadruped, withYawRotationUnconstrained)
{
  PoseOptimization optimization;

  optimization.setDesiredLegConfiguration( {
    Position(1.0, 0.5, 0.0),
    Position(1.0, -0.5, 0.0),
    Position(-1.0, -0.5, 0.0),
    Position(-1.0, 0.5, 0.0) });

  std::vector<Position> feetPositions;
  kindr::rotations::eigen_impl::EulerAnglesZyxPD rotation(0.5, 0.0, 0.0);
  feetPositions.push_back(rotation.inverseRotate(Position(2.0, 0.5, 0.0)));
  feetPositions.push_back(rotation.inverseRotate(Position(1.0, -0.5, 0.0)));
  feetPositions.push_back(rotation.inverseRotate(Position(-1.0, -0.5, 0.0)));
  feetPositions.push_back(rotation.inverseRotate(Position(-1.0, 0.5, 0.0)));
  optimization.setFeetPositions(feetPositions);

  Pose startPose;
  startPose.getRotation() = rotation;
  optimization.setStartPose(startPose);

  Pose result;
  EXPECT_TRUE(optimization.compute(result));

  Eigen::Matrix4d expected;
  expected << 0.9211, -0.3894, 0.0,  0.2194, // TODO Check.
              0.3894,  0.9211, 0.0,  0.1199,
              0.0,     0.0,    1.0,  0.0,
              0.0,     0.0,    0.0,  1.0;

//  0.82533561490967844 -0.56464247339503526        -0 0.21939564047259319
//  0.56464247339503526 0.82533561490967844         0 0.11985638465105081
//          0        -0         1         0
//          0         0         0         1

//  assertNear(expected, result.getTransformationMatrix(), 1e-3, KINDR_SOURCE_FILE_POS);
}

TEST(quadruped, constrained)
{
  PoseOptimization optimization;

  optimization.setDesiredLegConfiguration( {
    Position(1.0, 0.5, 0.0),
    Position(1.0, -0.5, 0.0),
    Position(-1.0, -0.5, 0.0),
    Position(-1.0, 0.5, 0.0) });

  std::vector<Position> feetPositions;
  kindr::rotations::eigen_impl::EulerAnglesXyzPD rotation(0.0, 0.0, 0.5);
  feetPositions.push_back(rotation.inverseRotate(Position(2.0, 0.5, 0.0)));
  feetPositions.push_back(rotation.inverseRotate(Position(1.0, -0.5, 0.0)));
  feetPositions.push_back(rotation.inverseRotate(Position(-1.0, -0.5, 0.0)));
  feetPositions.push_back(rotation.inverseRotate(Position(-1.0, 0.5, 0.0)));
  optimization.setFeetPositions(feetPositions);

  grid_map::Polygon supportPolygon;
  // Skipping LF.
  supportPolygon.addVertex(feetPositions[1].vector().head<2>());
  supportPolygon.addVertex(feetPositions[2].vector().head<2>());
  supportPolygon.addVertex(feetPositions[3].vector().head<2>());
  optimization.setSupportPolygon(supportPolygon);

  Pose startPose;
  startPose.getRotation() = rotation;
  optimization.setStartPose(startPose);

  Pose result;
  EXPECT_TRUE(optimization.compute(result));

  Eigen::Matrix4d expected;
  expected << 0.8253, -0.5646, 0.0,  0.2235,
              0.5646,  0.8253, 0.0,  0.0081,
              0.0,     0.0,    1.0,  0.0,
              0.0,     0.0,    0.0,  1.0;

  assertNear(expected, result.getTransformationMatrix(), 1e-3, KINDR_SOURCE_FILE_POS);
}
//
//
//x =
//
//    0.2235
//    0.0081
//   -0.1000
//
//
//T =
//
//    0.9211   -0.3894         0    0.2235
//    0.3894    0.9211         0    0.0081
//         0         0    1.0000         0
//         0         0         0    1.0000
