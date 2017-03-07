/*
 * BatchExecutor.hpp
 *
 *  Created on: Dec 20, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/executor/Executor.hpp"
#include "free_gait_core/executor/AdapterBase.hpp"
#include "free_gait_core/executor/StateBatch.hpp"

#include <thread>
#include <memory>
#include <functional>
#include <atomic>

namespace free_gait {

class BatchExecutor
{
 public:
  BatchExecutor(free_gait::Executor& executor);
  virtual ~BatchExecutor();

  void addProcessingCallback(std::function<void(bool)> callback);
  void setTimeStep(const double timeStep);
  double getTimeStep() const;
  bool process(const std::vector<free_gait::Step>& steps);
  bool isProcessing();
  void cancelProcessing();

  const StateBatch& getStateBatch() const;

 private:
  void processInThread();

  StateBatch stateBatch_;
  free_gait::Executor& executor_;

  std::function<void(bool)> callback_;
  double timeStep_;
  std::atomic<bool> isProcessing_;
  std::atomic<bool> requestForCancelling_;
};

} /* namespace free_gait_rviz_plugin */
