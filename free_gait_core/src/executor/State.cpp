/*
 * State.cpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/executor/State.hpp>
#include <free_gait_core/TypePrints.hpp>

#include <stdexcept>

namespace free_gait {

State::State()
    : QuadrupedState(),
      robotExecutionStatus_(false),
      feedForwardFrictionNorm_(0.0),
      isImpedanceTrajectory_(false)
{
}

State::~State()
{
}

void State::initialize(const std::vector<LimbEnum>& limbs, const std::vector<BranchEnum>& branches)
{
  for (const auto& limb : limbs) {
    isSupportLegs_[limb] = false;
    ignoreContact_[limb] = false;
    ignoreForPoseAdaptation_[limb] = false;
    endEffectorForceInWorldFrame_[limb] = Force(0.0, 0.0, 0.0);
  }

  for (const auto& branch : branches) {
    setEmptyControlSetup(branch);
  }
  feedForwardFrictionNorm_ = 0.0;
  isImpedanceTrajectory_ = false;
}

bool State::getRobotExecutionStatus() const
{
  return robotExecutionStatus_;
}

void State::setRobotExecutionStatus(bool robotExecutionStatus)
{
  robotExecutionStatus_ = robotExecutionStatus;
}

const std::string& State::getStepId() const
{
  return stepId_;
}

void State::setStepId(const std::string& stepId)
{
  stepId_ = stepId;
}

bool State::isSupportLeg(const LimbEnum& limb) const
{
  return isSupportLegs_.at(limb);
}

void State::setSupportLeg(const LimbEnum& limb, bool isSupportLeg)
{
  isSupportLegs_[limb] = isSupportLeg;
}

unsigned int State::getNumberOfSupportLegs() const
{
  unsigned int nLegs = 0;
  for (const auto& supportLeg : isSupportLegs_) {
    if (supportLeg.second) ++nLegs;
  }
  return nLegs;
}

bool State::isIgnoreContact(const LimbEnum& limb) const
{
  return ignoreContact_.at(limb);
}

void State::setIgnoreContact(const LimbEnum& limb, bool ignoreContact)
{
  ignoreContact_[limb] = ignoreContact;
}

bool State::hasSurfaceNormal(const LimbEnum& limb) const
{
  return (surfaceNormals_.count(limb) > 0u);
}

const Vector& State::getSurfaceNormal(const LimbEnum& limb) const
{
  return surfaceNormals_.at(limb);
}

void State::setSurfaceNormal(const LimbEnum& limb, const Vector& surfaceNormal)
{
  surfaceNormals_[limb] = surfaceNormal;
}

void State::removeSurfaceNormal(const LimbEnum& limb)
{
  surfaceNormals_.erase(limb);
}

bool State::isIgnoreForPoseAdaptation(const LimbEnum& limb) const
{
  return ignoreForPoseAdaptation_.at(limb);
}

void State::setIgnoreForPoseAdaptation(const LimbEnum& limb, bool ignorePoseAdaptation)
{
  ignoreForPoseAdaptation_[limb] = ignorePoseAdaptation;
}

const JointPositionsLeg State::getJointPositionsForLimb(const LimbEnum& limb) const
{
  return JointPositionsLeg(
      quadruped_model::QuadrupedState::getJointPositions().vector().segment<QD::getNumDofLimb()>(
          QD::getLimbStartIndexInJ(limb))
  );
}

void State::setEndEffectorPositionInWorldFrame(const LimbEnum& limb, const Position& endEffectorPositionInWorldFrame) {
  endEffectorPositionInWorldFrame_[limb] = endEffectorPositionInWorldFrame;
}

const Position& State::getEndEffectorPositionInWorldFrame(const LimbEnum& limb) const {
  return endEffectorPositionInWorldFrame_.at(limb);
}

void State::setEndEffectorVelocityInWorldFrame(const LimbEnum& limb, const LinearVelocity& endEffectorVelocityInWorldFrame) {
  endEffectorVelocityInWorldFrame_[limb] = endEffectorVelocityInWorldFrame;
}

const LinearVelocity& State::getEndEffectorVelocityInWorldFrame(const LimbEnum& limb) const {
  return endEffectorVelocityInWorldFrame_.at(limb);
}

void State::setEndEffectorForceInWorldFrame(const LimbEnum& limb, const Force& endEffectorForceInWorldFrame) {
  endEffectorForceInWorldFrame_[limb] = endEffectorForceInWorldFrame;
}

const Force& State::getEndEffectorForceInWorldFrame(const LimbEnum& limb) const {
  return endEffectorForceInWorldFrame_.at(limb);
}

void State::setJointPositionsForLimb(const LimbEnum& limb, const JointPositionsLeg& jointPositions)
{
  quadruped_model::QuadrupedState::getJointPositions().setSegment<QD::getNumDofLimb()>(
      QD::getLimbStartIndexInJ(limb), jointPositions);
}

void State::setAllJointPositions(const JointPositions& jointPositions)
{
  quadruped_model::QuadrupedState::setJointPositions(quadruped_model::JointPositions(jointPositions.vector()));
}

const JointVelocitiesLeg State::getJointVelocitiesForLimb(const LimbEnum& limb) const
{
  return JointVelocitiesLeg(
      quadruped_model::QuadrupedState::getJointVelocities().vector().segment<QD::getNumDofLimb()>(
      QD::getLimbStartIndexInJ(limb)
  ));
}

void State::setJointVelocitiesForLimb(const LimbEnum& limb, const JointVelocitiesLeg& jointVelocities)
{
  quadruped_model::QuadrupedState::getJointVelocities().setSegment<QD::getNumDofLimb()>(
      QD::getLimbStartIndexInJ(limb), jointVelocities);
}

void State::setAllJointVelocities(const JointVelocities& jointVelocities)
{
  quadruped_model::QuadrupedState::setJointVelocities(quadruped_model::JointVelocities(jointVelocities.vector()));
}

const JointAccelerationsLeg State::getJointAccelerationsForLimb(const LimbEnum& limb) const
{
  return JointAccelerationsLeg(jointAccelerations_.vector().segment<QD::getNumDofLimb()>(QD::getLimbStartIndexInJ(limb)));
}

const JointAccelerations& State::getAllJointAccelerations() const
{
  return jointAccelerations_;
}

void State::setJointAccelerationsForLimb(const LimbEnum& limb, const JointAccelerationsLeg& jointAccelerations)
{
  jointAccelerations_.setSegment<QD::getNumDofLimb()>(QD::getLimbStartIndexInJ(limb), jointAccelerations);
}

void State::setAllJointAccelerations(const JointAccelerations& jointAccelerations)
{
  jointAccelerations_ = jointAccelerations;
}

const JointEffortsLeg State::getJointEffortsForLimb(const LimbEnum& limb) const
{
  return JointEffortsLeg(getAllJointEfforts().vector().segment<QD::getNumDofLimb()>(QD::getLimbStartIndexInJ(limb)));
}

const JointEfforts& State::getAllJointEfforts() const
{
  return jointEfforts_;
}

void State::setJointEffortsForLimb(const LimbEnum& limb, const JointEffortsLeg& jointEfforts)
{
  jointEfforts_.setSegment<QD::getNumDofLimb()>(QD::getLimbStartIndexInJ(limb), jointEfforts);
}

void State::setAllJointEfforts(const JointEfforts& jointEfforts)
{
  jointEfforts_ = jointEfforts;
}

const ControlSetup& State::getControlSetup(const BranchEnum& branch) const
{
  return controlSetups_.at(branch);
}

const ControlSetup& State::getControlSetup(const LimbEnum& limb) const
{
  return controlSetups_.at(QD::mapEnums<QD::BranchEnum>(limb));
}

bool State::isControlSetupEmpty(const BranchEnum& branch) const
{
  for (const auto& level : controlSetups_.at(branch)) {
    if (level.second) return false;
  }
  return true;
}

bool State::isControlSetupEmpty(const LimbEnum& limb) const
{
  return isControlSetupEmpty(QD::mapEnums<QD::BranchEnum>(limb));
}

void State::setControlSetup(const BranchEnum& branch, const ControlSetup& controlSetup)
{
  controlSetups_[branch] = controlSetup;
}

void State::setControlSetup(const LimbEnum& limb, const ControlSetup& controlSetup)
{
  controlSetups_[QD::mapEnums<QD::BranchEnum>(limb)] = controlSetup;
}

void State::setEmptyControlSetup(const BranchEnum& branch)
{
  ControlSetup emptyControlSetup;
  emptyControlSetup[ControlLevel::Position] = false;
  emptyControlSetup[ControlLevel::Velocity] = false;
  emptyControlSetup[ControlLevel::Acceleration] = false;
  emptyControlSetup[ControlLevel::Effort] = false;
  controlSetups_[branch] = emptyControlSetup;
}

void State::setEmptyControlSetup(const LimbEnum& limb)
{
  setEmptyControlSetup(QD::mapEnums<QD::BranchEnum>(limb));
}

void State::getAllJointNames(std::vector<std::string>& jointNames) const
{
  jointNames.clear();
  jointNames.reserve(QD::getJointsDimension());
  for (const auto& jointKey : QD::getJointKeys()) {
    jointNames.push_back(QD::mapKeyEnumToKeyName(jointKey.getEnum()));
  }
}

const Vector& State::getImpedanceGainInWorldFrame(const ImpedanceControl& impedanceControlId) const {
  return impedanceGains_.at(impedanceControlId);
}

void State::setImpedanceGainInWorldFrame(const ImpedanceControl& impedanceControlId, const Vector& gain) {
  impedanceGains_[impedanceControlId] = gain;
}

double State::getFeedForwardFrictionNorm() const {
  return feedForwardFrictionNorm_;
}

void State::setFeedForwardFrictionNorm(double feedForwardFrictionNorm) {
  feedForwardFrictionNorm_ = feedForwardFrictionNorm;
}

bool State::getIsImpedanceTrajectory() const noexcept {
  return isImpedanceTrajectory_;
}

void State::setIsImpedanceTrajectory(bool isImpedanceTrajectory) noexcept {
  isImpedanceTrajectory_ = isImpedanceTrajectory;
}

std::ostream& operator<<(std::ostream& out, const State& state)
{
  out << static_cast<const quadruped_model::QuadrupedState&>(state) << std::endl;
  out << "Support legs: " << state.isSupportLegs_ << std::endl;
  out << "Ignore contact: " << state.ignoreContact_ << std::endl;
  out << "Ignore for pose adaptation: " << state.ignoreForPoseAdaptation_ << std::endl;
  out << "Control setup:" << std::endl;
  for (const auto& controlSetup : state.controlSetups_) {
    out << controlSetup.first << ": ";
    for (const auto& controlLevel : controlSetup.second) {
      if (controlLevel.second) out << controlLevel.first << ", ";
    }
    out << std::endl;
  }
  out << "Surface normals: " << state.surfaceNormals_ << std::endl;
  if (!state.stepId_.empty()) out << "Step ID: " << state.stepId_ << std::endl;
  return out;
}

} /* namespace free_gait */
