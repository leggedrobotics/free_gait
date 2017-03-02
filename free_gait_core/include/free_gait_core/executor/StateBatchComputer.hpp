/*
 * StateBatchComputer.hpp
 *
 *  Created on: Dec 22, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include <free_gait_core/executor/StateBatch.hpp>
#include <free_gait_core/executor/AdapterBase.hpp>

namespace free_gait {

class StateBatchComputer
{
 public:
  StateBatchComputer(std::shared_ptr<AdapterBase> adapter);
  virtual ~StateBatchComputer();

  void computeEndEffectorTrajectories(StateBatch& stateBatch);
  void computeEndEffectorTargets(StateBatch& stateBatch);
  void computeBaseTrajectories(StateBatch& stateBatch);

 private:
  std::shared_ptr<AdapterBase> adapter_;
};

} /* namespace free_gait */
