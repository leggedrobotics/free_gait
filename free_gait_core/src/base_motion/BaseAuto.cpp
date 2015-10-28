/*
 * BaseAuto.cpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/base_motion/BaseAuto.hpp"
#include "free_gait_core/base_motion/BaseMotionBase.hpp"
#include "free_gait_core/leg_motion/EndEffectorMotionBase.hpp"

// TODO Move.
#include "loco/terrain_perception/TerrainPerceptionFreePlane.hpp"
#include "roco/log/log_messages.hpp"

#include <math.h>

namespace free_gait {

BaseAuto::BaseAuto()
    : BaseMotionBase(BaseMotionBase::Type::Auto),
      averageLinearVelocity_(0.0),
      averageAngularVelocity_(0.0),
      duration_(0.0),
      height_(0.0),
      supportMargin_(0.0),
      computed_(false),
      controlSetup_ { {ControlLevel::Position, true}, {ControlLevel::Velocity, false},
                      {ControlLevel::Acceleration, false}, {ControlLevel::Force, false} }
{
}

BaseAuto::~BaseAuto()
{
}

std::unique_ptr<BaseMotionBase> BaseAuto::clone() const
{
  std::unique_ptr<BaseMotionBase> pointer(new BaseAuto(*this));
  return pointer;
}

const ControlSetup BaseAuto::getControlSetup() const
{
  return controlSetup_;
}

bool BaseAuto::compute(const State& state, const Step& step, const AdapterBase& adapter)
{
  if (!generateFootholdLists(state, step, adapter)) {
    std::cerr << "BaseAuto::compute(): Could not generate foothold lists." << std::endl;
    return false;
  }
  Position horizontalTargetPositionInWorldFrame;
  getAdaptiveHorizontalTargetPosition(state, adapter, horizontalTargetPositionInWorldFrame);
  getAdaptiveTargetPose(state, adapter, horizontalTargetPositionInWorldFrame, target_);
  if (!optimizePose(target_)) {
    std::cerr << "BaseAuto::compute(): Could not compute pose optimization." << std::endl;
    return false;
  }
  computeDuration();
  computeTrajectory();
  return computed_ = true;
}

void BaseAuto::updateStartPose(const Pose& startPose)
{
  computed_ = false;
  start_.getPosition() = startPose.getPosition();
  start_.getRotation() = startPose.getRotation().getUnique();
}

Pose BaseAuto::evaluatePose(const double time) const
{
  double timeInRange = time <= duration_ ? time : duration_;
  return trajectory_.evaluate(timeInRange);
}

Twist BaseAuto::evaluateTwist(const double time) const
{
  double timeInRange = time <= duration_ ? time : duration_;
  return trajectory_.evaluateDerivative(timeInRange, 1);
}

double BaseAuto::getDuration() const
{
  return duration_;
}

bool BaseAuto::generateFootholdLists(const State& state, const Step& step, const AdapterBase& adapter)
{
  // Footholds for orientation.
//  footholdsForOrientation_.clear();
//  for (auto leg : *legs_) {
//    const auto& limb = quadrupedModel_->getLimbEnumFromLimbUInt((uint) leg->getId());
//    footholdsForOrientation_.emplace(limb, leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame());
//  }

  footholdsInSupport_.clear();
  for (const auto& limb : state.getLimbs()) {
    if (state.isSupportLeg(limb)) {
      footholdsInSupport_[limb] = adapter.getPositionWorldToFootInWorldFrame(limb);
    }
  }

  footholdsToReach_.clear();
  for (const auto& limb : state.getLimbs()) {
    if (!state.isIgnoreForPoseAdaptation(limb)) {
      if (step.hasLegMotion(limb)) {
        // Double check if right format.
        if (step.getLegMotion(limb).getTrajectoryType() == LegMotionBase::TrajectoryType::EndEffector
            && step.getLegMotion(limb).getControlSetup().at(ControlLevel::Position)) {
          // Use target end effector position.
          const auto& legMotion = dynamic_cast<const EndEffectorMotionBase&>(step.getLegMotion(limb));
          footholdsToReach_[limb] = legMotion.getTargetPosition();
        }
      } else {
        // Use current foot position.
        footholdsToReach_[limb] = adapter.getPositionWorldToFootInWorldFrame(limb);
      }
    }
  }

  nominalStanceInBaseFrame_.clear();
  for (const auto& stance : nominalPlanarStanceInBaseFrame_) {
    nominalStanceInBaseFrame_.emplace(stance.first, Position(stance.second(0), stance.second(1), -height_));
  }

  return true;
}

void BaseAuto::getAdaptiveHorizontalTargetPosition(const State& state, const AdapterBase& adapter, Position& horizontalTargetPositionInWorldFrame)
{
  std::vector<double> legWeights(state.getLimbs().size());
  for (auto& weight : legWeights)
    weight = 1.0;
  double sumWeights = 0;
  int iLeg = 0;

  for (const auto& limb : state.getLimbs()) {
    if (state.isSupportLeg(limb)) legWeights[iLeg] = 0.2;
    sumWeights += legWeights[iLeg];
    iLeg++;
  }

  if (sumWeights != 0) {
    iLeg = 0;
    for (const auto& limb : state.getLimbs()) {
      horizontalTargetPositionInWorldFrame += adapter.getPositionWorldToFootInWorldFrame(limb)
          * legWeights[iLeg];
      iLeg++;
    }
    horizontalTargetPositionInWorldFrame /= sumWeights;
  } else {
    for (const auto& limb : state.getLimbs()) {
      horizontalTargetPositionInWorldFrame += adapter.getPositionWorldToFootInWorldFrame(limb);
    }
    horizontalTargetPositionInWorldFrame /= state.getLimbs().size();
  }

  horizontalTargetPositionInWorldFrame.z() = 0.0;
}

void BaseAuto::getAdaptiveTargetPose(
    const State& state, const AdapterBase& adapter, const Position& horizontalTargetPositionInWorld, Pose& targetPoseInWorld)
{
  // TODO Cleanup and move to pose optimizer.

  // Get terrain from target foot positions.
  loco::TerrainModelFreePlane terrain;
  std::vector<Position> footholds; // TODO
  for (const auto& limb : state.getLimbs()) {
    footholds.push_back(adapter.getPositionWorldToFootInWorldFrame(limb));
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

bool BaseAuto::optimizePose(Pose& pose)
{
  poseOptimization_.setStance(footholdsToReach_);
  poseOptimization_.setNominalStance(nominalStanceInBaseFrame_);

  grid_map::Polygon support;
  for (auto foothold : footholdsInSupport_) {
    support.addVertex(foothold.second.vector().head<2>());
  }
  support.offsetInward(supportMargin_);
  poseOptimization_.setSupportPolygon(support);

  return poseOptimization_.optimize(pose);
}

void BaseAuto::computeDuration()
{
  double distance = (target_.getPosition() - start_.getPosition()).norm();
  double translationDuration = distance / averageLinearVelocity_;
  double angle = fabs(target_.getRotation().getDisparityAngle(start_.getRotation()));
  double rotationDuration = angle / averageAngularVelocity_;
  duration_ = translationDuration > rotationDuration ? translationDuration : rotationDuration;
}

bool BaseAuto::computeTrajectory()
{
  std::vector<Time> times;
  std::vector<ValueType> values;

  times.push_back(0.0);
  values.push_back(start_);

  times.push_back(duration_);
  values.push_back(target_);

  trajectory_.fitCurve(times, values);
  return true;
}

std::ostream& operator<<(std::ostream& out, const BaseAuto& baseAuto)
{
  out << "Height: " << baseAuto.height_ << std::endl;
  out << "Average Linear Velocity: " << baseAuto.averageLinearVelocity_ << std::endl;
  out << "Average Angular Velocity: " << baseAuto.averageAngularVelocity_ << std::endl;
  out << "Support Margin: " << baseAuto.supportMargin_ << std::endl;
  out << "Duration: " << baseAuto.duration_ << std::endl;
  out << "Start Position: " << baseAuto.start_.getPosition() << std::endl;
  out << "Start Orientation: " << baseAuto.start_.getRotation() << std::endl;
  out << "Start Orientation (yaw, pitch, roll) [deg]: " << 180.0 / M_PI * EulerAnglesZyx(baseAuto.start_.getRotation()).getUnique().vector().transpose() << std::endl;
  out << "Target Position: " << baseAuto.target_.getPosition() << std::endl;
  out << "Target Orientation: " << baseAuto.target_.getRotation() << std::endl;
  out << "Target Orientation (yaw, pitch, roll) [deg]: " << 180.0 / M_PI * EulerAnglesZyx(baseAuto.target_.getRotation()).getUnique().vector().transpose() << std::endl;
  return out;
}

} /* namespace */

