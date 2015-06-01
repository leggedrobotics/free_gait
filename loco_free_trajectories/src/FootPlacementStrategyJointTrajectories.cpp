/*
 * FootPlacementStrategyJointTrajectories.hpp
 *
 *  Created on: Mar 14, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/foot_placement_strategy/FootPlacementStrategyJointTrajectories.hpp"
#include "loco/common/TorsoBase.hpp"

// Roco
#include <roco/log/log_messages.hpp>

// Robot model
#include "robot_model/RobotModel_common.hpp"

namespace loco {

FootPlacementStrategyJointTrajectories::FootPlacementStrategyJointTrajectories(std::shared_ptr<LegGroup> legs,
                                                             std::shared_ptr<TorsoBase> torso,
                                                             std::shared_ptr<StepQueue> stepQueue)
    : FootPlacementStrategyBase(),
      legs_(legs),
      torso_(torso),
      stepQueue_(stepQueue)
{

}

FootPlacementStrategyJointTrajectories::~FootPlacementStrategyJointTrajectories() {

}

bool FootPlacementStrategyJointTrajectories::loadParameters(const TiXmlHandle& handle)
{
  return true;
}

bool FootPlacementStrategyJointTrajectories::initialize(double dt)
{
  for (auto leg : *legs_) {
    leg->setDesiredJointPositions(leg->getMeasuredJointPositions());
    loco::LegBase::JointControlModes jointControlModes;
    jointControlModes.toImplementation().setConstant(robot_model::AM_Position);
    leg->setDesiredJointControlModes(jointControlModes);
  }
  return true;
}

bool FootPlacementStrategyJointTrajectories::advance(double dt)
{
  if (stepQueue_->empty()) return true;
  for (auto leg : *legs_) generateDesiredFootPosition(leg);
  return true;
}

void FootPlacementStrategyJointTrajectories::generateDesiredFootPosition(LegBase* leg) const
{
  auto& step = stepQueue_->getCurrentStep(); // Already checked if step available above.
  if (!step.hasSwingData(leg->getName())) return;
  auto trajectory = step.getSwingData().at(leg->getName()).getTrajectory();

  if (trajectory->getFrameId() != "base") {
    ROCO_WARN_STREAM_FP("Only trajectories in base frame supported.");
    return;
  }

  Position startPosition = leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame();
  trajectory->updateStartPosition(startPosition);
  Position desiredPosition = Position(trajectory->evaluate(leg->getSwingPhase()));
  setJointPositions(leg, desiredPosition);
}

void FootPlacementStrategyJointTrajectories::setJointPositions(
    LegBase* leg, const Position& footPositionInBaseFrame) const
{
  leg->setDesiredJointPositions(
      leg->getJointPositionsFromPositionBaseToFootInBaseFrame(footPositionInBaseFrame));
}

bool FootPlacementStrategyJointTrajectories::setToInterpolated(
    const FootPlacementStrategyBase& footPlacementStrategy1,
    const FootPlacementStrategyBase& footPlacementStrategy2, double t)
{
  throw std::runtime_error("FootPlacementStrategyJointTrajectories::setToInterpolated not implemented yet!");
  return false;
}

const LegGroup& FootPlacementStrategyJointTrajectories::getLegs() const {
  return *legs_;
}

} // namespace loco

