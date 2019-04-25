/*
 * ExecutorState.hpp
 *
 *  Created on: Mar 8, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include <cstddef>

namespace free_gait {

class ExecutorState
{
 public:
  ExecutorState();
  ExecutorState(size_t stepNumber, double stepPhase);
  virtual ~ExecutorState();

  void setState(size_t stepNumber, double stepPhase);
  size_t stepNumber() const;
  double stepPhase() const;

  friend bool operator>(const ExecutorState& stateA, const ExecutorState& stateB);
  friend bool operator<=(const ExecutorState& stateA, const ExecutorState& stateB);
  friend bool operator<(const ExecutorState& stateA, const ExecutorState& stateB);
  friend bool operator>=(const ExecutorState& stateA, const ExecutorState& stateB);

 private:
  size_t stepNumber_;
  double stepPhase_;
};

} /* namespace free_gait */
