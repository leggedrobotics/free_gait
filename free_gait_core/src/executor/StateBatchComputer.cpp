/*
 * StateBatchComputer.cpp
 *
 *  Created on: Dec 22, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */
#include <free_gait_core/executor/StateBatchComputer.hpp>

namespace free_gait {

StateBatchComputer::StateBatchComputer(AdapterBase& adapter) :
    adapter_(adapter)
{
}

StateBatchComputer::~StateBatchComputer()
{
}

void StateBatchComputer::computeEndEffectorTargets(StateBatch& stateBatch)
{
  stateBatch.endEffectorTargets_.clear();
  stateBatch.endEffectorTargets_.resize(adapter_.getLimbs().size());

  size_t i = 0;
  for (const auto& limb : adapter_.getLimbs()) {
    const State* previousState = &(stateBatch.getStates().begin()->second);
    for (const auto& state : stateBatch.getStates()) {
      if (!previousState->isSupportLeg(limb) && state.second.isSupportLeg(limb)) {
        // Switching to support leg.
        adapter_.setInternalDataFromState(state.second);
        const Position position = adapter_.getPositionWorldToFootInWorldFrame(limb);
        stateBatch.endEffectorTargets_[i][state.first] = position;
      }
      previousState = &state.second;
    }
    i++;
  }
}

void StateBatchComputer::computeEndEffectorTrajectories(StateBatch& stateBatch)
{
  stateBatch.endEffectorPositions_.clear();
  stateBatch.endEffectorPositions_.resize(adapter_.getLimbs().size());
  for (const auto& state : stateBatch.getStates()) {
    adapter_.setInternalDataFromState(state.second);
    size_t i = 0;
    for (const auto& limb : adapter_.getLimbs()) {
      const Position position = adapter_.getPositionWorldToFootInWorldFrame(limb);
      stateBatch.endEffectorPositions_[i++][state.first] = position;
    }
  }
}

void StateBatchComputer::computeStances(StateBatch& stateBatch)
{
  stateBatch.stances_.clear();

  // Add first stance.
  const auto firstState = stateBatch.getStates().begin();
  adapter_.setInternalDataFromState(firstState->second);
  Stance stance;
  for (const auto& limb : adapter_.getLimbs()) {
    if (!firstState->second.isSupportLeg(limb)) continue;
    stance[limb] = adapter_.getPositionWorldToFootInWorldFrame(limb);
  }
  stateBatch.stances_[firstState->first] = stance;

  // Other stances.
  const State* previousState = &(firstState->second);
  for (const auto& state : stateBatch.getStates()) {
    for (const auto& limb : adapter_.getLimbs()) {
      if (previousState->isSupportLeg(limb) != state.second.isSupportLeg(limb)) {
        // New stance.
        adapter_.setInternalDataFromState(state.second);
        Stance stance;
        for (const auto& limb : adapter_.getLimbs()) {
          if (!state.second.isSupportLeg(limb)) continue;
          stance[limb] = adapter_.getPositionWorldToFootInWorldFrame(limb);
        }
        stateBatch.stances_[state.first] = stance;
      }
    }
    previousState = &state.second;
  }
}

} /* namespace free_gait */
