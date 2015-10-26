/*
 * AdapterBase.hpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/executor/State.hpp"
#include "free_gait_core/step/StepQueue.hpp"
#include <memory>

namespace free_gait {

class AdapterBase
{
 public:
  AdapterBase();
  virtual ~AdapterBase();
  virtual void initializeState(State& state) const  = 0;
  virtual bool updateStateWithMeasurements(State& state) const = 0;
  virtual bool updateExtras(const StepQueue& stepQueue, State& state) const = 0;
  virtual Position getPositionWorldToFootInWorldFrame(const LimbEnum& limb) const = 0;
};

} /* namespace free_gait */
