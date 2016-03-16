/*
 * StepComputer.hpp
 *
 *  Created on: Mar 16, 2016
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/step/StepComputer.hpp"

namespace free_gait {

StepComputer::StepComputer() :
    isDone_(false)
{
}

StepComputer::~StepComputer()
{
}

bool StepComputer::initialize()
{
  return true;
}

bool StepComputer::compute()
{
  const bool result = step_.compute();
  isDone_ = true;
  return result;
}

bool StepComputer::isBusy()
{
  return false;
}

bool StepComputer::isDone()
{
  return isDone_;
}

void StepComputer::resetIsDone()
{
  isDone_ = false;
}

void StepComputer::setStep(const Step& step)
{
  step_ = step;
}

void StepComputer::getStep(Step& step)
{
  step = step_;
}

} /* namespace */

