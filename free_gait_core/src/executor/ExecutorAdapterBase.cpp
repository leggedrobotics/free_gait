/*
 * ExecutorAdapterBase.cpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/executor/ExecutorAdapterBase.hpp>

namespace free_gait {

ExecutorAdapterBase::ExecutorAdapterBase(std::shared_ptr<ExecutorState> state)
    : state_(state)
{
}

ExecutorAdapterBase::~ExecutorAdapterBase()
{
}

std::shared_ptr<ExecutorState> ExecutorAdapterBase::getState()
{
  return state_;
}

} /* namespace free_gait */
