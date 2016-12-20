/*
 * StateBatch.cpp
 *
 *  Created on: Dec 20, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/executor/StateBatch.hpp>

namespace free_gait {

StateBatch::StateBatch()
{
}

StateBatch::~StateBatch()
{
}

const std::map<double, State>& StateBatch::getStates() const
{
  return states_;
}

void StateBatch::addState(const double time, const State& state)
{
  states_[time] = state;
}

const State& StateBatch::getState(const double time)
{
  std::map<double, State>::iterator iterator = states_.lower_bound(time);
  return (*iterator).second;
}

void StateBatch::clear()
{
  states_.clear();
}

} /* namespace free_gait */
