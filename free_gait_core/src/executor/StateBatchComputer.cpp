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
  stateBatch.endEffectorPositions_.clear();
  stateBatch.endEffectorPositions_.resize(adapter_->getLimbs().size());
  for (const auto& state : stateBatch.getStates()) {
    adapter_->setInternalDataFromState(state.second);
    size_t i = 0;
    for (const auto& limb : adapter_->getLimbs()) {
      const Position position = adapter_->getPositionWorldToFootInWorldFrame(limb);
      stateBatch.endEffectorPositions_[i++][state.first] = position;
    }
  }
}

} /* namespace free_gait */
