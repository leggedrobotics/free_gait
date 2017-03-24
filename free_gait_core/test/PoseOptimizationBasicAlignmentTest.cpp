/*
 * PoseOptimizationTest.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationBasicAlignment.hpp"

// gtest
#include <gtest/gtest.h>

// kindr
#include <kindr/Core>
#include <kindr/common/gtest_eigen.hpp>

// Grid map
#include <grid_map_core/Polygon.hpp>

using namespace free_gait;

TEST(PoseOptimizationBasicAlignment, quadrupedSymmetricUnconstrained)
{
  PoseOptimizationBasicAlignment optimization;

  optimization.setNominalStance( Stance({
    {LimbEnum::LF_LEG, Position(1.0, 0.5, -0.4)},
    {LimbEnum::RF_LEG, Position(1.0, -0.5, -0.4)},
    {LimbEnum::LH_LEG, Position(-1.0, -0.5, -0.4)},
    {LimbEnum::RH_LEG, Position(-1.0, 0.5, -0.4)} }));

  optimization.setStance( Stance({
    {LimbEnum::LF_LEG, Position(1.0, 0.5, -0.1)},
    {LimbEnum::RF_LEG, Position(1.0, -0.5, -0.1)},
    {LimbEnum::LH_LEG, Position(-1.0, -0.5, -0.1)},
    {LimbEnum::RH_LEG, Position(-1.0, 0.5, -0.1)} }));

  Pose result;
  ASSERT_TRUE(optimization.optimize(result));

  Eigen::Vector3d expectedPosition(0.0, 0.0, 0.3);
  RotationMatrix expectedOrientation; // Identity.

  kindr::assertNear(expectedOrientation.matrix(), RotationMatrix(result.getRotation()).matrix(), 1e-2, KINDR_SOURCE_FILE_POS);
  kindr::assertNear(expectedPosition, result.getPosition().vector(), 1e-3, KINDR_SOURCE_FILE_POS);
}

TEST(PoseOptimizationBasicAlignment, quadrupedSymmetricWithOffsetUnconstrained)
{
  PoseOptimizationBasicAlignment optimization;

  optimization.setNominalStance( Stance({
    {LimbEnum::LF_LEG, Position(1.0, 0.5, 0.0)},
    {LimbEnum::RF_LEG, Position(1.0, -0.5, 0.0)},
    {LimbEnum::LH_LEG, Position(-1.0, -0.5, 0.0)},
    {LimbEnum::RH_LEG, Position(-1.0, 0.5, 0.0)} }));

  optimization.setStance( Stance({
    {LimbEnum::LF_LEG, Position(31.0, 20.5, 10.0)},
    {LimbEnum::RF_LEG, Position(31.0, 19.5, 10.0)},
    {LimbEnum::LH_LEG, Position(29.0, 19.5, 10.0)},
    {LimbEnum::RH_LEG, Position(29.0, 20.5, 10.0)} }));

  Pose startPose;
  startPose.getPosition() << 30.0, 20.0, 10.0;

  Pose result = startPose;
  ASSERT_TRUE(optimization.optimize(result));

  kindr::assertNear(startPose.getTransformationMatrix(), result.getTransformationMatrix(), 1e-3, KINDR_SOURCE_FILE_POS);
}

TEST(PoseOptimizationBasicAlignment, quadrupedAsymmetricUnconstrained)
{
  PoseOptimizationBasicAlignment optimization;

  optimization.setNominalStance( Stance({
    {LimbEnum::LF_LEG, Position(1.0, 0.5, 0.0)},
    {LimbEnum::RF_LEG, Position(1.0, -0.5, 0.0)},
    {LimbEnum::LH_LEG, Position(-1.0, -0.5, 0.0)},
    {LimbEnum::RH_LEG, Position(-1.0, 0.5, 0.0)} }));

  optimization.setStance( Stance({
    {LimbEnum::LF_LEG, Position(2.0, 0.5, 0.0)},
    {LimbEnum::RF_LEG, Position(1.0, -0.5, 0.0)},
    {LimbEnum::LH_LEG, Position(-1.0, -0.5, 0.0)},
    {LimbEnum::RH_LEG, Position(-1.0, 0.5, 0.0)} }));

  Pose result;
  ASSERT_TRUE(optimization.optimize(result));

  Eigen::Matrix3d expectedOrientation;
  expectedOrientation <<  1.0, 0.1, 0.0,
                          -0.1,  1.0, 0.0,
                          0.0,  0.0, 1.0;

  Eigen::Vector3d expectedPosition(0.25, 0.0, 0.0);

  kindr::assertNear(expectedOrientation, RotationMatrix(result.getRotation()).matrix(), 1e-2, KINDR_SOURCE_FILE_POS);
  kindr::assertNear(expectedPosition, result.getPosition().vector(), 1e-3, KINDR_SOURCE_FILE_POS);
}

TEST(PoseOptimizationBasicAlignment, quadrupedSymmetricWithYawUnconstrained)
{
  PoseOptimizationBasicAlignment optimization;

  optimization.setNominalStance( Stance({
    {LimbEnum::LF_LEG, Position(1.0, 0.5, 0.0)},
    {LimbEnum::RF_LEG, Position(1.0, -0.5, 0.0)},
    {LimbEnum::LH_LEG, Position(-1.0, -0.5, 0.0)},
    {LimbEnum::RH_LEG, Position(-1.0, 0.5, 0.0)} }));

  Stance footPositions;
  kindr::EulerAnglesZyxPD rotation(0.5, 0.0, 0.0);
  footPositions[LimbEnum::LF_LEG] = rotation.rotate(Position( 1.0,  0.5, 0.0));
  footPositions[LimbEnum::RF_LEG] = rotation.rotate(Position(1.0,  -0.5, 0.0));
  footPositions[LimbEnum::LH_LEG] = rotation.rotate(Position(-1.0, -0.5, 0.0));
  footPositions[LimbEnum::RH_LEG] = rotation.rotate(Position(-1.0, 0.5, 0.0));
  optimization.setStance(footPositions);

  Pose startPose;
  startPose.getRotation() = rotation;

  Pose result = startPose;
  ASSERT_TRUE(optimization.optimize(result));

  kindr::assertNear(startPose.getTransformationMatrix(), result.getTransformationMatrix(), 1e-3, KINDR_SOURCE_FILE_POS);
}

//
//TEST(quadruped, withYawUnconstrained)
//{
//  PoseOptimization optimization;
//
//  optimization.setDesiredLegConfiguration( {
//    Position(1.0, 0.5, 0.0),
//    Position(1.0, -0.5, 0.0),
//    Position(-1.0, -0.5, 0.0),
//    Position(-1.0, 0.5, 0.0) });
//
//  std::vector<Position> feetPositions;
//  kindr::rotations::eigen_impl::EulerAnglesZyxPD rotation(0.5, 0.0, 0.0);
//  feetPositions.push_back(rotation.inverseRotate(Position(2.0, 0.5, 0.0)));
//  feetPositions.push_back(rotation.inverseRotate(Position(1.0, -0.5, 0.0)));
//  feetPositions.push_back(rotation.inverseRotate(Position(-1.0, -0.5, 0.0)));
//  feetPositions.push_back(rotation.inverseRotate(Position(-1.0, 0.5, 0.0)));
//  optimization.setFeetPositions(feetPositions);
//
//  Pose pose;
//  pose.getRotation() = rotation;
//  ASSERT_TRUE(optimization.optimize(pose));
//
//  Eigen::Matrix3d expectedOrientation;
//  expectedOrientation <<  0.9255, 0.3917, 0.0,
//                         -0.3917, 0.9255, 0.0,
//                          0.0,    0.0,    1.0;
//
//  Eigen::Vector3d expectedPosition(0.2194, 0.1199, 0.0);
//
//  assertNear(expectedOrientation, RotationMatrix(pose.getRotation()).matrix(), 1e-2, KINDR_SOURCE_FILE_POS);
//  assertNear(expectedPosition, pose.getPosition().vector(), 1e-3, KINDR_SOURCE_FILE_POS);
//}
//
TEST(PoseOptimizationBasicAlignment, constrained)
{
  PoseOptimizationBasicAlignment optimization;

  optimization.setNominalStance( Stance({
    {LimbEnum::LF_LEG, Position(1.0, 0.5, 0.0)},
    {LimbEnum::RF_LEG, Position(1.0, -0.5, 0.0)},
    {LimbEnum::LH_LEG, Position(-1.0, -0.5, 0.0)},
    {LimbEnum::RH_LEG, Position(-1.0, 0.5, 0.0)} }));

  Stance feetPositions;
  kindr::EulerAnglesZyxPD rotation(0.0, 0.0, 0.5);
  feetPositions[LimbEnum::LF_LEG] = rotation.rotate(Position(2.0, 0.5, 0.0));
  feetPositions[LimbEnum::LH_LEG] = rotation.rotate(Position(-1.0, 0.5, 0.0));
  feetPositions[LimbEnum::RH_LEG] = rotation.rotate(Position(-1.0, -0.5, 0.0));
  feetPositions[LimbEnum::RF_LEG] = rotation.rotate(Position(1.0, -0.5, 0.0));
  optimization.setStance(feetPositions);

  grid_map::Polygon supportPolygon;
  // Skipping LF.
  supportPolygon.addVertex(feetPositions[LimbEnum::LH_LEG].vector().head<2>());
  supportPolygon.addVertex(feetPositions[LimbEnum::RH_LEG].vector().head<2>());
  supportPolygon.addVertex(feetPositions[LimbEnum::RF_LEG].vector().head<2>());
  optimization.setSupportRegion(supportPolygon);

  Pose pose;
  pose.getRotation() = rotation;
  ASSERT_TRUE(optimization.optimize(pose));

  supportPolygon.offsetInward(-1e-5);
  ASSERT_TRUE(supportPolygon.isInside(pose.getPosition().vector().head<2>()));

//  Eigen::Matrix3d expectedOrientation;
//  expectedOrientation <<  0.9255, 0.3917, 0.0,
//                         -0.3917, 0.9255, 0.0,
//                          0.0,    0.0,    1.0;
//
//  Eigen::Vector3d expectedPosition(0.2235, 0.0081, 0.0);
//
//  kindr::assertNear(expectedOrientation, RotationMatrix(pose.getRotation()).matrix(), 1e-2, KINDR_SOURCE_FILE_POS);
//  kindr::assertNear(expectedPosition, pose.getPosition().vector(), 1e-3, KINDR_SOURCE_FILE_POS);
}
//
//TEST(quadruped, DebugLoco1)
//{
//  PoseOptimization optimization;
//
//  optimization.setDesiredLegConfiguration( {
//    Position(0.2525, 0.185, -0.36),
//    Position(0.2525, -0.185, -0.36),
//    Position(-0.2525, -0.185, -0.36),
//    Position(-0.2525, 0.185, -0.36) });
//
//  optimization.setFeetPositions({
//    Position(0.328993, 0.220197, 0.0137381),
//    Position(0.328873, -0.21858, 0.0137441),
//    Position(-0.30376, 0.219006, 0.013784),
//    Position(-0.303636, -0.217628, 0.0137876) });
//
//  Position expectedPosition(0.0126176, 0.000748497, 0.373763);
//  RotationMatrix expectedOrientation;
//  expectedOrientation.matrix() << 1.0, -0.000188893, 7.06191e-05, 0.000188892, 1.0, 1.10137e-05, -7.06212e-05, -1.10003e-05, 1.0;
//
//  Pose pose;
//  pose.getPosition() = expectedPosition;
//  pose.getRotation() = expectedOrientation;
//  ASSERT_TRUE(optimization.optimize(pose));
//
//  assertNear(expectedOrientation.matrix(), RotationMatrix(pose.getRotation()).matrix(), 1e-2, KINDR_SOURCE_FILE_POS);
//  assertNear(expectedPosition.vector(), pose.getPosition().vector(), 1e-3, KINDR_SOURCE_FILE_POS);
//}
