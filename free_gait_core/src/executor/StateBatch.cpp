/*
 * StateBatch.cpp
 *
 *  Created on: Dec 20, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/executor/StateBatch.hpp>

#include <functional>
#include <stdexcept>

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

std::vector<std::map<double, Position>> StateBatch::getEndEffectorPositions() const
{
  return endEffectorPositions_;
}

std::vector<std::map<double, Position>> StateBatch::getEndEffectorTargets() const
{
  return endEffectorTargets_;
}

std::vector<std::map<double, std::tuple<Position, Vector>>>StateBatch::getSurfaceNormals() const
{
  return surfaceNormals_;
}

std::map<double, Stance> StateBatch::getStances() const
{
  return stances_;
}

bool StateBatch::getEndTimeOfStep(std::string& stepId, double& endTime) const
{
  if (stepIds_.empty()) return false;
  for (std::map<double, std::string>::const_iterator it = stepIds_.begin(); it != stepIds_.end(); ++it) {
    if (it->second == stepId) {
      ++it;
      if (it == stepIds_.end()) {
        endTime = getEndTime();
      } else {
        endTime = it->first;
      }
      return true;
    }
  }
  return false;
}

void StateBatch::addState(const double time, const State& state)
{
  states_[time] = state;
}

bool StateBatch::isValidTime(const double time) const
{
  if (states_.empty()) return false;
  if (time < states_.begin()->first) return false;
  return true;
}

double free_gait::StateBatch::getStartTime() const
{
  if (states_.empty()) throw std::out_of_range("State batch error: Batch is empty.");
  return states_.begin()->first;
}

double free_gait::StateBatch::getEndTime() const
{
  if (states_.empty()) throw std::out_of_range("State batch error: Batch is empty.");
  return (--states_.end())->first;
}

const State& StateBatch::getState(const double time) const
{
  if (!isValidTime(time)) throw std::out_of_range("State batch error: No state available for requested time.");
  if (time > (--states_.end())->first) return (--states_.end())->second;
  return states_.lower_bound(time)->second;
}

void StateBatch::clear()
{
  states_.clear();
}

} /* namespace free_gait */
