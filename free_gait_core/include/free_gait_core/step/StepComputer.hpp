/*
 * StepComputer.hpp
 *
 *  Created on: Mar 16, 2016
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/step/Step.hpp"

namespace free_gait {

class StepComputer
{
 public:
  StepComputer();
  virtual ~StepComputer();
  virtual bool initialize();
  virtual bool compute();
  virtual bool isBusy();
  virtual bool isDone();
  virtual void resetIsDone();
  void setStep(const Step& step);
  void getStep(Step& step);

 protected:
  Step step_;
  bool isDone_;
};

} /* namespace */

