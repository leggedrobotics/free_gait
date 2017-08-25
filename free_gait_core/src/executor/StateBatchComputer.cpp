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

void StateBatchComputer::computeEndEffectorTargetsAndSurfaceNormals(StateBatch& stateBatch)
{
  stateBatch.endEffectorTargets_.clear();
  stateBatch.endEffectorTargets_.resize(adapter_.getLimbs().size());
  stateBatch.surfaceNormals_.clear();
  stateBatch.surfaceNormals_.resize(adapter_.getLimbs().size());

  // Surface normal at start.
  size_t i = 0;
  for (const auto& limb : adapter_.getLimbs()) {
    const double time = stateBatch.getStates().begin()->first;
    const State& state = stateBatch.getStates().begin()->second;
    if (state.hasSurfaceNormal(limb)) {
      adapter_.setInternalDataFromState(state);
      const Position position = adapter_.getPositionWorldToFootInWorldFrame(limb);
      std::get<0>(stateBatch.surfaceNormals_[i][time]) = position;
      std::get<1>(stateBatch.surfaceNormals_[i][time]) = state.getSurfaceNormal(limb);
    }
    i++;
  }

  // Iterate trough states.
  i = 0;
  for (const auto& limb : adapter_.getLimbs()) {
    const State* previousState = &(stateBatch.getStates().begin()->second);
    for (const auto& state : stateBatch.getStates()) {
      if (!previousState->isSupportLeg(limb) && state.second.isSupportLeg(limb)) {
        // Switching to support leg.
        adapter_.setInternalDataFromState(state.second);
        const Position position = adapter_.getPositionWorldToFootInWorldFrame(limb);
        stateBatch.endEffectorTargets_[i][state.first] = position;
        if (state.second.hasSurfaceNormal(limb)) {
          std::get<0>(stateBatch.surfaceNormals_[i][state.first]) = position;
          std::get<1>(stateBatch.surfaceNormals_[i][state.first]) = state.second.getSurfaceNormal(limb);
        }
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

void StateBatchComputer::computeStepIds(StateBatch& stateBatch)
{
  stateBatch.stepIds_.clear();
  for (const auto& state : stateBatch.getStates()) {
    const std::string& stepId(state.second.getStepId());
    if (stepId.empty()) continue;
    if (stateBatch.stepIds_.size() == 0) {
      stateBatch.stepIds_[state.first] = stepId;
      continue;
    }
    if (std::prev(stateBatch.stepIds_.end())->second != stepId) {
      stateBatch.stepIds_[state.first] = stepId;
      continue;
    }
  }
}

} /* namespace free_gait */
