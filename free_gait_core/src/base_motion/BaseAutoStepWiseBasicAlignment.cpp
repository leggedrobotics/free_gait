/*
 * BaseAutoStepWiseBasicAlignmentStepWiseBasicAlignment.cpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/base_motion/BaseAutoStepWiseBasicAlignment.hpp"
#include "free_gait_core/base_motion/BaseMotionBase.hpp"
#include "free_gait_core/leg_motion/EndEffectorMotionBase.hpp"

// TODO Move.
#include "loco/terrain_perception/TerrainPerceptionFreePlane.hpp"

#include <math.h>

namespace free_gait {

BaseAutoStepWiseBasicAlignment::BaseAutoStepWiseBasicAlignment()
    : BaseMotionBase(BaseMotionBase::Type::Auto),
      ignoreTimingOfLegMotion_(false),
      averageLinearVelocity_(0.0),
      averageAngularVelocity_(0.0),
      duration_(0.0),
      supportMargin_(0.0),
      minimumDuration_(0.0),
      isComputed_(false),
      controlSetup_ { {ControlLevel::Position, true}, {ControlLevel::Velocity, true},
                      {ControlLevel::Acceleration, false}, {ControlLevel::Effort, false} }
{
}

BaseAutoStepWiseBasicAlignment::~BaseAutoStepWiseBasicAlignment()
{
}

BaseAutoStepWiseBasicAlignment::BaseAutoStepWiseBasicAlignment(const BaseAutoStepWiseBasicAlignment& other) :
    BaseMotionBase(other),
    ignoreTimingOfLegMotion_(other.ignoreTimingOfLegMotion_),
    averageLinearVelocity_(other.averageLinearVelocity_),
    averageAngularVelocity_(other.averageAngularVelocity_),
    supportMargin_(other.supportMargin_),
    minimumDuration_(other.minimumDuration_),
    start_(other.start_),
    target_(other.target_),
    duration_(other.duration_),
    nominalPlanarStanceInBaseFrame_(other.nominalPlanarStanceInBaseFrame_),
    controlSetup_(other.controlSetup_),
    trajectory_(other.trajectory_),
    footholdsToReach_(other.footholdsToReach_),
    footholdsInSupport_(other.footholdsInSupport_),
    nominalStanceInBaseFrame_(other.nominalStanceInBaseFrame_),
    footholdsForTerrain_(other.footholdsForTerrain_),
    footholdsForOrientation_(other.footholdsForOrientation_),
    poseOptimization_(other.poseOptimization_),
    isComputed_(other.isComputed_)
{
  if (other.height_) height_.reset(new double(*(other.height_)));
}

std::unique_ptr<BaseMotionBase> BaseAutoStepWiseBasicAlignment::clone() const
{
  std::unique_ptr<BaseMotionBase> pointer(new BaseAutoStepWiseBasicAlignment(*this));
  return pointer;
}

const ControlSetup BaseAutoStepWiseBasicAlignment::getControlSetup() const
{
  return controlSetup_;
}

void BaseAutoStepWiseBasicAlignment::updateStartPose(const Pose& startPose)
{
  isComputed_ = false;
  start_ = startPose;
}

bool BaseAutoStepWiseBasicAlignment::prepareComputation(const State& state, const Step& step, const StepQueue& queue, const AdapterBase& adapter)
{
  if (!height_) {
    if (!computeHeight(state, queue, adapter)) {
      std::cerr << "BaseAutoStepWiseBasicAlignment::compute: Could not compute height." << std::endl;
      return false;
    }
  }
  if (!generateFootholdLists(state, step, queue, adapter)) {
    std::cerr << "BaseAutoStepWiseBasicAlignment::compute: Could not generate foothold lists." << std::endl;
    return false;
  }
  Position horizontalTargetPositionInWorldFrame;
  getAdaptiveHorizontalTargetPosition(state, adapter, horizontalTargetPositionInWorldFrame);
  getAdaptiveTargetPose(state, adapter, horizontalTargetPositionInWorldFrame, target_);
  if (!optimizePose(target_)) {
    std::cerr << "BaseAutoStepWiseBasicAlignment::compute: Could not compute pose optimization." << std::endl;
    return false;
  }
  computeDuration(step, adapter);
  computeTrajectory();
  return isComputed_ = true;
}

bool BaseAutoStepWiseBasicAlignment::needsComputation() const
{
  return false;
}

bool BaseAutoStepWiseBasicAlignment::isComputed() const
{
  return isComputed_;
}

Pose BaseAutoStepWiseBasicAlignment::evaluatePose(const double time) const
{
  double timeInRange = time <= duration_ ? time : duration_;
  Pose pose;
  trajectory_.evaluate(pose, timeInRange);
  return pose;
}

Twist BaseAutoStepWiseBasicAlignment::evaluateTwist(const double time) const
{
  double timeInRange = time <= duration_ ? time : duration_;
  curves::CubicHermiteSE3Curve::DerivativeType derivative;
  trajectory_.evaluateDerivative(derivative, timeInRange, 1);
  Twist twist(derivative.getTranslationalVelocity().vector(),
              derivative.getRotationalVelocity().vector());
  return twist;
}

double BaseAutoStepWiseBasicAlignment::getDuration() const
{
  return duration_;
}

const std::string& BaseAutoStepWiseBasicAlignment::getFrameId(const ControlLevel& controlLevel) const
{
  return frameId_;
}

void BaseAutoStepWiseBasicAlignment::setHeight(const double height)
{
  height_.reset(new double(height));
}

double BaseAutoStepWiseBasicAlignment::getHeight() const
{
  if (height_) return *height_;
  throw std::runtime_error("Height of BaseAutoStepWiseBasicAlignment has not been set yet.");
}

void BaseAutoStepWiseBasicAlignment::setAverageLinearVelocity(const double averageLinearVelocity)
{
  averageLinearVelocity_ = averageLinearVelocity;
}

double BaseAutoStepWiseBasicAlignment::getAverageLinearVelocity() const
{
  return averageLinearVelocity_;
}

bool BaseAutoStepWiseBasicAlignment::computeHeight(const State& state, const StepQueue& queue, const AdapterBase& adapter)
{
  if (queue.previousStepExists()) {
    if (queue.getPreviousStep().hasBaseMotion()) {
      if (queue.getPreviousStep().getBaseMotion().getType() == BaseMotionBase::Type::Auto) {
        const auto& previousBaseMotion = dynamic_cast<const BaseAutoStepWiseBasicAlignment&>(queue.getPreviousStep().getBaseMotion());
        height_.reset(new double(previousBaseMotion.getHeight()));
        return true;
      }
    }
  }

  unsigned n = 0;
  double heightSum = 0;
  for (const auto& limb : adapter.getLimbs()) {
    if (!state.isIgnoreForPoseAdaptation(limb)) {
      double legHeight = -adapter.getPositionBaseToFootInBaseFrame(limb, state.getJointPositionsForLimb(limb)).z();
      heightSum += legHeight;
      ++n;
    }
  }
  if (n == 0) return false;
  height_.reset(new double(heightSum / (double)(n)));
  return true;
}

bool BaseAutoStepWiseBasicAlignment::generateFootholdLists(const State& state, const Step& step, const StepQueue& queue, const AdapterBase& adapter)
{
  footholdsInSupport_.clear();
  bool prepareForNextStep = false;
  if (!step.hasLegMotion() && queue.size() > 1) {
    if (queue.getNextStep().hasLegMotion()) prepareForNextStep = true;
  }

  if (prepareForNextStep) {
    // Auto motion for preparation of next step.
    for (const auto& limb : adapter.getLimbs()) {
      if (!state.isIgnoreContact(limb) && !queue.getNextStep().hasLegMotion(limb)) {
        footholdsInSupport_[limb] = adapter.getPositionWorldToFootInWorldFrame(limb);
      }
    }
  } else {
    // Auto motion for current step.
    for (const auto& limb : adapter.getLimbs()) {
      if (!step.hasLegMotion(limb) && !state.isIgnoreContact(limb)) {
        footholdsInSupport_[limb] = adapter.getPositionWorldToFootInWorldFrame(limb);
      }
    }
  }

  footholdsToReach_.clear();
  for (const auto& limb : adapter.getLimbs()) {
    if (step.hasLegMotion(limb)) {
      if (step.getLegMotion(limb).isIgnoreForPoseAdaptation()) continue;
      // Double check if right format.
      if (step.getLegMotion(limb).getTrajectoryType() == LegMotionBase::TrajectoryType::EndEffector
          && step.getLegMotion(limb).getControlSetup().at(ControlLevel::Position)) {
        // Use target end effector position.
        const auto& legMotion = dynamic_cast<const EndEffectorMotionBase&>(step.getLegMotion(limb));
        footholdsToReach_[limb] = adapter.transformPosition(legMotion.getFrameId(ControlLevel::Position),
                                                            adapter.getWorldFrameId(),
                                                            legMotion.getTargetPosition());
      }
      else {
        return false;
      }
    } else {
      // Use current foot position.
      if (!state.isIgnoreForPoseAdaptation(limb)) {
        footholdsToReach_[limb] = adapter.getPositionWorldToFootInWorldFrame(limb);
      }
    }
  }

  // TODO: Delete as soon full pose optimization is done.
  footholdsForTerrain_ = footholdsToReach_;

  footholdsForOrientation_ = footholdsToReach_;
  for (const auto& limb : adapter.getLimbs()) {
    if (footholdsForOrientation_.count(limb) == 0) footholdsForOrientation_[limb] = adapter.getPositionWorldToFootInWorldFrame(limb);
  }

  nominalStanceInBaseFrame_.clear();
  for (const auto& stance : nominalPlanarStanceInBaseFrame_) {
    nominalStanceInBaseFrame_.emplace(stance.first, Position(stance.second(0), stance.second(1), -*height_));
  }

  return true;
}

void BaseAutoStepWiseBasicAlignment::getAdaptiveHorizontalTargetPosition(const State& state, const AdapterBase& adapter, Position& horizontalTargetPositionInWorldFrame)
{
  std::vector<double> legWeights(adapter.getLimbs().size());
  for (auto& weight : legWeights)
    weight = 1.0;
  double sumWeights = 0;
  int iLeg = 0;

  for (const auto& limb : adapter.getLimbs()) {
    if (state.isSupportLeg(limb)) legWeights[iLeg] = 0.2;
    sumWeights += legWeights[iLeg];
    iLeg++;
  }

  if (sumWeights != 0) {
    iLeg = 0;
    for (const auto& limb : adapter.getLimbs()) {
      horizontalTargetPositionInWorldFrame += adapter.getPositionWorldToFootInWorldFrame(limb)
          * legWeights[iLeg];
      iLeg++;
    }
    horizontalTargetPositionInWorldFrame /= sumWeights;
  } else {
    for (const auto& limb : adapter.getLimbs()) {
      horizontalTargetPositionInWorldFrame += adapter.getPositionWorldToFootInWorldFrame(limb);
    }
    horizontalTargetPositionInWorldFrame /= adapter.getLimbs().size();
  }

  horizontalTargetPositionInWorldFrame.z() = 0.0;
}

void BaseAutoStepWiseBasicAlignment::getAdaptiveTargetPose(
    const State& state, const AdapterBase& adapter, const Position& horizontalTargetPositionInWorld, Pose& targetPoseInWorld)
{
  // TODO Cleanup and move to pose optimizer.

  // Get terrain from target foot positions.
  loco::TerrainModelFreePlane terrain;
  std::vector<Position> footholds; // TODO
  for (const auto& foothold : footholdsForTerrain_) {
    footholds.push_back(foothold.second);
  }
  loco::TerrainPerceptionFreePlane::generateTerrainModelFromPointsInWorldFrame(footholds, terrain);
  loco::Vector terrainNormalInWorld;
  terrain.getNormal(loco::Position::Zero(), terrainNormalInWorld);

  // Compute orientation of terrain in world.
  double terrainPitch, terrainRoll;
  const double adaptationFactor = 0.7;
  terrainPitch = adaptationFactor * atan2(terrainNormalInWorld.x(), terrainNormalInWorld.z()); // TODO Replace with better handling of rotations.
  terrainRoll = adaptationFactor * atan2(terrainNormalInWorld.y(), terrainNormalInWorld.z());
  RotationQuaternion orientationWorldToTerrain = RotationQuaternion(AngleAxis(terrainRoll, -1.0, 0.0, 0.0))
      * RotationQuaternion(AngleAxis(terrainPitch, 0.0, 1.0, 0.0));

  // Compute target height over terrain and determine target position.
  Position positionWorldToDesiredHeightAboveTerrainInTerrain(0.0, 0.0, *height_);
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
  const Position positionForeFeetMidPointInWorld = (footholdsForOrientation_[LimbEnum::LF_LEG] + footholdsForOrientation_[LimbEnum::RF_LEG]) * 0.5;
  const Position positionHindFeetMidPointInWorld = (footholdsForOrientation_[LimbEnum::LH_LEG] + footholdsForOrientation_[LimbEnum::RH_LEG]) * 0.5;
  Vector desiredHeadingDirectionInWorld = Vector(
      positionForeFeetMidPointInWorld - positionHindFeetMidPointInWorld);
  desiredHeadingDirectionInWorld.z() = 0.0;
  RotationQuaternion desiredHeading;
  desiredHeading.setFromVectors(Vector::UnitX().toImplementation(), desiredHeadingDirectionInWorld.toImplementation()); // Why is toImplementation() required here?

  // Create target pose.
  targetPoseInWorld.getPosition() = targetPositionInWorld;
  targetPoseInWorld.getRotation() = orientationWorldToTerrain * desiredHeading; // TODO Correct??
}

bool BaseAutoStepWiseBasicAlignment::optimizePose(Pose& pose)
{
  poseOptimization_.setStance(footholdsToReach_);
  poseOptimization_.setNominalStance(nominalStanceInBaseFrame_);

  grid_map::Polygon support;
  std::vector<Position> footholdsOrdered;
  getFootholdsCounterClockwiseOrdered(footholdsInSupport_, footholdsOrdered);
  for (auto foothold : footholdsOrdered) {
    support.addVertex(foothold.vector().head<2>());
  }
  support.offsetInward(supportMargin_);
  poseOptimization_.setSupportRegion(support);

  return poseOptimization_.optimize(pose);
}

void BaseAutoStepWiseBasicAlignment::computeDuration(const Step& step, const AdapterBase& adapter)
{
  if (!step.hasLegMotion() || ignoreTimingOfLegMotion_) {
    double distance = (target_.getPosition() - start_.getPosition()).norm();
    double translationDuration = distance / averageLinearVelocity_;
    double angle = fabs(target_.getRotation().getDisparityAngle(start_.getRotation()));
    double rotationDuration = angle / averageAngularVelocity_;
    duration_ = translationDuration > rotationDuration ? translationDuration : rotationDuration;
  } else {
    for (const auto& limb : adapter.getLimbs()) {
      if (step.hasLegMotion(limb)) {
        if (!step.getLegMotion(limb).isIgnoreForPoseAdaptation()) {
          if (duration_ < step.getLegMotion(limb).getDuration()) duration_ = step.getLegMotion(limb).getDuration();
        }
      }
    }
  }

  duration_ = duration_ < minimumDuration_ ? minimumDuration_ : duration_;
}

bool BaseAutoStepWiseBasicAlignment::computeTrajectory()
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

std::ostream& operator<<(std::ostream& out, const BaseAutoStepWiseBasicAlignment& baseAuto)
{
  out << "Frame: " << baseAuto.frameId_ << std::endl;
  out << "Height: " << *(baseAuto.height_) << std::endl;
  out << "Ignore timing of leg motion: " << (baseAuto.ignoreTimingOfLegMotion_ ? "True" : "False") << std::endl;
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
  out << "Footholds in support: ";
  for (const auto& f : baseAuto.footholdsInSupport_) out << f.first << " ";
  out << std::endl;
  out << "Footholds to reach: ";
  for (const auto& f : baseAuto.footholdsToReach_) out << f.first << " ";
  out << std::endl;
  return out;
}

} /* namespace */
