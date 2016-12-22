/*
 * StateBatchComputer.cpp
 *
 *  Created on: Dec 22, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */
#include <free_gait_core/executor/StateBatchComputer.hpp>

namespace free_gait {

StateBatchComputer::StateBatchComputer(std::shared_ptr<AdapterBase> adapter) :
    adapter_(adapter)
{
}

StateBatchComputer::~StateBatchComputer()
{
}

void StateBatchComputer::computeEndEffectorTrajectories(StateBatch& stateBatch)
{
  for (const auto& state : stateBatch.getStates()) {
    adapter_->setInternalDataFromState(state.second);
    for (const auto& limb : adapter_->getLimbs()) {
      const Position position = adapter_->getPositionWorldToFootInWorldFrame(limb);
      stateBatch.endEffectorPositions_[state.first] = position;
    }
  }
}

} /* namespace free_gait */
