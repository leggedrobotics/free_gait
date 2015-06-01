/*
 * GaitPatternJointTrajectories.cpp
 *
 *  Created on: Mar 14, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/gait_pattern/GaitPatternJointTrajectories.hpp"

#include "tinyxml.h"

#include <cmath>
#include <vector>
#include <stdexcept>
#include <limits>

// Loco
#include "loco/utils/math.hpp"

namespace loco {

GaitPatternJointTrajectories::GaitPatternJointTrajectories(std::shared_ptr<LegGroup> legs,
                                         std::shared_ptr<TorsoBase> torso,
                                         std::shared_ptr<StepQueue> stepQueue)
    : isInitialized_(false),
      torso_(torso),
      legs_(legs),
      stepQueue_(stepQueue)
{

}

GaitPatternJointTrajectories::~GaitPatternJointTrajectories()
{
}

double GaitPatternJointTrajectories::getSwingPhaseForLeg(int iLeg)
{
  if (stepQueue_->empty()) return -1.0;
  Step& step = stepQueue_->getCurrentStep();
  if (step.getState() == Step::State::PreStep || step.getState() == Step::State::PostStep) return -1.0;
  if (!step.hasSwingData(legs_->getLegById(iLeg)->getName())) return -1.0;
  return step.getAtStepPhase(legs_->getLegById(iLeg)->getName());
}

double GaitPatternJointTrajectories::getStanceDuration(int iLeg)
{
  return 0.0;
}

double GaitPatternJointTrajectories::getStrideDuration()
{
  throw std::runtime_error("GaitPatternFreeGait::getStrideDuration not implemented yet!");
  return 0.0;
}

void GaitPatternJointTrajectories::setStrideDuration(double strideDuration)
{
  throw std::runtime_error("GaitPatternFreeGait::setStrideDuration not implemented yet!");
}

double GaitPatternJointTrajectories::getStancePhaseForLeg(int iLeg)
{
  return 0.0;
}

bool GaitPatternJointTrajectories::loadParameters(const TiXmlHandle &hParameterSet)
{
  TiXmlElement* pElem;

  pElem = hParameterSet.FirstChild("GaitPattern").Element();
  if (!pElem) {
    printf("Could not find GaitPattern\n");
    return false;
  }

  return true;
}

bool GaitPatternJointTrajectories::initialize(double dt)
{
  isInitialized_ = true;
  return isInitialized_;
}

bool GaitPatternJointTrajectories::saveParameters(TiXmlHandle &hParameterSet)
{
  throw std::runtime_error("GaitPatternFreeGait::saveParameters not implemented yet!");
  return false;
}

unsigned long int GaitPatternJointTrajectories::getNGaitCycles()
{
  throw std::runtime_error("GaitPatternFreeGait::getNGaitCycles not implemented yet!");
  return 0;
}

bool GaitPatternJointTrajectories::advance(double dt)
{
  for (auto leg : *legs_) {
    leg->setShouldBeGrounded(shouldBeLegGrounded(leg->getId()));
    leg->setSwingDuration(getSwingDuration(leg->getId()));
    leg->setSwingPhase(getSwingPhaseForLeg(leg->getId()));
    leg->setStanceDuration(getStanceDuration(leg->getId()));
    leg->setStancePhase(getStancePhaseForLeg(leg->getId()));
  }
  return true;
}

bool GaitPatternJointTrajectories::shouldBeLegGrounded(int iLeg)
{
  return false;
}

double GaitPatternJointTrajectories::getStridePhase() const
{
  throw std::runtime_error("GaitPatternFreeGait::getStridePhase not implemented yet!");
  return 0.0;
}

void GaitPatternJointTrajectories::setStridePhase(double stridePhase)
{
  throw std::runtime_error("GaitPatternFreeGait::setStridePhase not implemented yet!");
}

bool GaitPatternJointTrajectories::isInitialized()
{
  return isInitialized_;
}

double GaitPatternJointTrajectories::getTimeLeftInStance(int iLeg, double strideDuration,
                                                double stridePhase) const
{
  throw std::runtime_error("GaitPatternFreeGait::getTimeLeftInStance not implemented yet!");
  return 0.0;
}

double GaitPatternJointTrajectories::getTimeLeftInSwing(int iLeg, double strideDuration,
                                               double stridePhase) const
{
  throw std::runtime_error("GaitPatternFreeGait::getTimeLeftInSwing not implemented yet!");
  return 0.0;
}

double GaitPatternJointTrajectories::getTimeSpentInStance(int iLeg, double strideDuration,
                                                 double stridePhase) const
{
  throw std::runtime_error("GaitPatternFreeGait::getTimeSpentInStance not implemented yet!");
  return 0.0;
}

double GaitPatternJointTrajectories::getTimeSpentInSwing(int iLeg, double strideDuration,
                                                double stridePhase) const
{
  throw std::runtime_error("GaitPatternFreeGait::getTimeSpentInSwing not implemented yet!");
  return 0.0;
}

double GaitPatternJointTrajectories::getTimeUntilNextStancePhase(int iLeg, double strideDuration,
                                                        double stridePhase) const
{
  throw std::runtime_error("GaitPatternFreeGait::getTimeUntilNextStancePhase not implemented yet!");
  return 0.0;
}

double GaitPatternJointTrajectories::getTimeUntilNextSwingPhase(int iLeg, double strideDuration,
                                                       double stridePhase) const
{
  throw std::runtime_error("GaitPatternFreeGait::getTimeUntilNextSwingPhase not implemented yet!");
  return 0.0;
}

double GaitPatternJointTrajectories::getSwingDuration(int iLeg, double strideDuration) const
{
  if (stepQueue_->empty()) return 0.0;
  Step& step = stepQueue_->getCurrentStep();
  return step.getAtStepDuration(legs_->getLegById(iLeg)->getName());
}

double GaitPatternJointTrajectories::getStanceDuration(int iLeg, double strideDuration) const
{
  throw std::runtime_error("GaitPatternFreeGait::getStanceDuration not implemented yet!");
  return 0.0;
}

double GaitPatternJointTrajectories::getSwingPhaseForLeg(int iLeg, double stridePhase) const
{
  throw std::runtime_error("GaitPatternFreeGait::getSwingPhaseForLeg not implemented yet!");
  return 0.0;
}

double GaitPatternJointTrajectories::getStancePhaseForLeg(int iLeg, double stridePhase) const
{
  throw std::runtime_error("GaitPatternFreeGait::getStancePhaseForLeg not implemented yet!");
  return 0.0;
}

int GaitPatternJointTrajectories::getNumberOfStanceLegs(double stridePhase)
{
  throw std::runtime_error("GaitPatternFreeGait::getNumberOfStanceLegs not implemented yet!");
  return 0;
}

}  // namespace loco
