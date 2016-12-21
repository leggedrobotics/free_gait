/*
 * StateBatch.hpp
 *
 *  Created on: Dec 20, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <free_gait_core/executor/State.hpp>

// STD
#include <map>

namespace free_gait {

class StateBatch
{
 public:
  StateBatch();
  virtual ~StateBatch();

  const std::map<double, State>& getStates() const;
  void addState(const double time, const State& state);
  bool isValidTime(const double time) const;
  double getBeginTime() const;
  double getEndTime() const;
  const State& getState(const double time) const;
  void clear();

 private:
  std::map<double, State> states_;
};

} /* namespace free_gait */
