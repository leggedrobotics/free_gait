/*
 * BaseAuto.cpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/base_motion/BaseAuto.hpp"
#include "free_gait_core/base_motion/BaseMotionBase.hpp"

// TODO Move.
#include "loco/terrain_perception/TerrainPerceptionFreePlane.hpp"
#include "roco/log/log_messages.hpp"

#include <math.h>

namespace free_gait {

BaseAuto::BaseAuto(const State& state, const Step& step, const AdapterBase& adapter)
    : BaseMotionBase(BaseMotionBase::Type::Auto),
      state_(state),
      step_(step),
      adapter_(adapter),
      hasTarget_(false),
      averageVelocity_(0.0),
      duration_(0.0),
      height_(0.0),
      supportSafetyMargin_(0.0),
      trajectoryUpdated_(false),
      controlSetup_ { {ControlLevel::Position, true}, {ControlLevel::Velocity, false},
                      {ControlLevel::Acceleration, false}, {ControlLevel::Force, false} }
{

}

BaseAuto::~BaseAuto()
{
}

const ControlSetup BaseAuto::getControlSetup() const
{
  return controlSetup_;
}

bool BaseAuto::compute()
{
  generateFootholdLists();
  // TODO !!!
}

void BaseAuto::updateStartPose(const Pose& startPose)
{
  start_.getPosition() = startPose.getPosition();
  start_.getRotation() = startPose.getRotation().getUnique();
  computeTrajectory();
}

Pose BaseAuto::evaluatePose(const double time) const
{
  double timeInRange = time <= duration_ ? time : duration_;
  return Pose(trajectory_.evaluate(timeInRange));
}

double BaseAuto::getDuration() const
{
  return duration_;
}

bool BaseAuto::generateFootholdLists()
{
  // Footholds for orientation.
//  footholdsForOrientation_.clear();
//  for (auto leg : *legs_) {
//    const auto& limb = quadrupedModel_->getLimbEnumFromLimbUInt((uint) leg->getId());
//    footholdsForOrientation_.emplace(limb, leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame());
//  }

  footholdsInSupport_.clear();
  for (const auto& limb : state_.getLimbs()) {
    if (state_.isSupportLeg(limb))
      footholdsInSupport_[limb] = adapter_.getPositionWorldToFootInWorldFrame(limb);
    if (!state_.isIgnoreForPoseAdaptation(limb))
      footholdsToReach_[limb] = adapter_.getPositionWorldToFootInWorldFrame(limb);

  }

  return true;
}

void BaseAuto::getAdaptiveHorizontalTargetPosition(Position& horizontalTargetPositionInWorldFrame)
{
  std::vector<double> legWeights(state_.getLimbs().size());
  for (auto& weight : legWeights)
    weight = 1.0;
  double sumWeights = 0;
  int iLeg = 0;

  for (const auto& limb : state_.getLimbs()) {
    if (state_.isSupportLeg(limb)) legWeights[iLeg] = 0.2;
    sumWeights += legWeights[iLeg];
    iLeg++;
  }

  if (sumWeights != 0) {
    iLeg = 0;
    for (const auto& limb : state_.getLimbs()) {
      horizontalTargetPositionInWorldFrame += adapter_.getPositionWorldToFootInWorldFrame(limb)
          * legWeights[iLeg];
      iLeg++;
    }
    horizontalTargetPositionInWorldFrame /= sumWeights;
  } else {
    for (const auto& limb : state_.getLimbs()) {
      horizontalTargetPositionInWorldFrame += adapter_.getPositionWorldToFootInWorldFrame(limb);
    }
    horizontalTargetPositionInWorldFrame /= state_.getLimbs().size();
  }

  horizontalTargetPositionInWorldFrame.z() = 0.0;
}

void BaseAuto::getAdaptiveTargetPose(
    const Position& horizontalTargetPositionInWorld, Pose& targetPoseInWorld)
{
  // TODO Cleanup and move to pose optimizer.

  // Get terrain from target foot positions.
  loco::TerrainModelFreePlane terrain;
  std::vector<Position> footholds; // TODO
  for (const auto& limb : state_.getLimbs()) {
    footholds.push_back(adapter_.getPositionWorldToFootInWorldFrame(limb));
  }
  loco::TerrainPerceptionFreePlane::generateTerrainModelFromPointsInWorldFrame(footholds, terrain);
  loco::Vector terrainNormalInWorld;
  terrain.getNormal(loco::Position::Zero(), terrainNormalInWorld);

  // Compute orientation of terrain in world.
  double terrainPitch, terrainRoll;
  terrainPitch = atan2(terrainNormalInWorld.x(), terrainNormalInWorld.z()); // TODO Replace with better handling of rotations.
  terrainRoll = atan2(terrainNormalInWorld.y(), terrainNormalInWorld.z());
  RotationQuaternion orientationWorldToTerrain = RotationQuaternion(AngleAxis(terrainRoll, -1.0, 0.0, 0.0))
      * RotationQuaternion(AngleAxis(terrainPitch, 0.0, 1.0, 0.0));

  // Compute target height over terrain and determine target position.
  Position positionWorldToDesiredHeightAboveTerrainInTerrain(0.0, 0.0, height_);
  Position positionWorldToDesiredHeightAboveTerrainInWorld = orientationWorldToTerrain.inverseRotate(positionWorldToDesiredHeightAboveTerrainInTerrain);
  double heightOverTerrain = positionWorldToDesiredHeightAboveTerrainInWorld.dot(terrainNormalInWorld);
  heightOverTerrain /= terrainNormalInWorld.z();
  double heightOfTerrainInWorld;
  terrain.getHeight(horizontalTargetPositionInWorld, heightOfTerrainInWorld);
  Position targetPositionInWorld = horizontalTargetPositionInWorld
      + (heightOfTerrainInWorld + heightOverTerrain) * Position::UnitZ();

  // Compute target orientation.
  // This is the center of the (target) feet projected on the x-y plane of the world frame.
//  loco::Position centerOfFeetInWorld;
//  for (const auto& footPosition : footPositions) {
//    centerOfFeetInWorld += footPosition;
//  }
//  centerOfFeetInWorld /= footPositions.size();
//  centerOfFeetInWorld.z() = 0.0;

  // Get desired heading direction with respect to the target feet.
  const Position positionForeFeetMidPointInWorld = (footholds[0] + footholds[1]) * 0.5; // TODO
  const Position positionHindFeetMidPointInWorld = (footholds[2] + footholds[3]) * 0.5; // TODO
  Vector desiredHeadingDirectionInWorld = Vector(
      positionForeFeetMidPointInWorld - positionHindFeetMidPointInWorld);
  desiredHeadingDirectionInWorld.z() = 0.0;
  RotationQuaternion desiredHeading;
  desiredHeading.setFromVectors(Vector::UnitX().toImplementation(), desiredHeadingDirectionInWorld.toImplementation()); // Why is toImplementation() required here?

  // Create target pose.
  targetPoseInWorld.getPosition() = targetPositionInWorld;
  targetPoseInWorld.getRotation() = desiredHeading * orientationWorldToTerrain;
}

void BaseAuto::optimizePose(Pose& pose)
{
  poseOptimization_.setFeetPositions(footholdsToReach_);
  poseOptimization_.setDesiredLegConfiguration(desiredFeetPositionsInBase_);

  grid_map::Polygon support;
  for (auto foothold : footholdsInSupport_) {
    support.addVertex(foothold.second.vector().head<2>());
  }
  support.offsetInward(supportSafetyMargin_);
  poseOptimization_.setSupportPolygon(support);

  if (!poseOptimization_.optimize(pose))
    ROCO_WARN_STREAM("Could not compute free gait pose optimization.");
}

bool BaseAuto::computeTrajectory()
{
  if (!hasTarget_) return false;

  std::vector<Time> times;
  std::vector<ValueType> values;

  times.push_back(0.0);
  values.push_back(start_);

  times.push_back(duration_);
  values.push_back(target_);

  trajectory_.fitCurve(times, values);
  trajectoryUpdated_ = true;
  return true;
}

std::ostream& operator<<(std::ostream& out, const BaseAuto& baseAuto)
{
//  out << "Start Position: " << baseShiftProfile.start_.getPosition() << std::endl;
//  out << "Start Orientation: " << baseShiftProfile.start_.getRotation() << std::endl;
//  out << "Start Orientation (yaw, pitch, roll) [deg]: " << 180.0 / M_PI * EulerAnglesZyx(baseShiftProfile.start_.getRotation()).getUnique().vector().transpose() << std::endl;
//  out << "Target Position: " << baseShiftProfile.target_.getPosition() << std::endl;
//  out << "Target Orientation: " << baseShiftProfile.target_.getRotation() << std::endl;
//  out << "Target Orientation (yaw, pitch, roll) [deg]: " << 180.0 / M_PI * EulerAnglesZyx(baseShiftProfile.target_.getRotation()).getUnique().vector().transpose() << std::endl;
//  out << "Height: " << baseShiftProfile.height_ << std::endl;
//  out << "Duration: " << baseShiftProfile.duration_ << std::endl;
//  out << "Type: " << baseShiftProfile.profileType_;
  return out;
}

} /* namespace */

