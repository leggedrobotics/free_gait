/*
 * FreeGaitPreviewPlayback.hpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include <free_gait_core/free_gait_core.hpp>
#include <free_gait_ros/free_gait_ros.hpp>

// ROS
#include <ros/ros.h>

// STD
#include <thread>
#include <memory>
#include <functional>
#include <mutex>

namespace free_gait_rviz_plugin {

class FreeGaitPreviewPlayback
{
 public:
  enum class PlayMode {
    STOPPED,
    FORWARD,
    BACKWARD,
    ONHOLD
  };

  typedef std::lock_guard<std::recursive_mutex> Lock;

  FreeGaitPreviewPlayback(ros::NodeHandle& nodeHandle,
                          std::shared_ptr<free_gait::AdapterBase> adapter);
  virtual ~FreeGaitPreviewPlayback();

  void addNewGoalCallback(std::function<void()> callback);
  void addStateChangedCallback(std::function<void(const ros::Time&)> callback);
  void addReachedEndCallback(std::function<void()> callback);

  bool process(const std::vector<free_gait::Step>& steps);
  bool isProcessing();
  void cancelProcessing();

  void run();
  void stop();
  void goToTime(const ros::Time& time);
  void clear();

  const free_gait::StateBatch& getStateBatch() const;
  void update(double timeStep);

 private:
  void processingCallback(bool success);
  void publish(const ros::Time& time);

  ros::NodeHandle& nodeHandle_;
  std::function<void()> newGoalCallback_;
  std::function<void(const ros::Time&)> stateChangedCallback_;
  std::function<void()> reachedEndCallback_;
  std::unique_ptr<free_gait::BatchExecutor> batchExecutor_;
  std::shared_ptr<free_gait::State> executorState_;
  free_gait::StateBatch stateBatch_;
  free_gait::StateBatchComputer stateBatchComputer_;
  std::recursive_mutex dataMutex_;
  free_gait::StateRosPublisher stateRosPublisher_;
  PlayMode playMode_;
  ros::Time time_;
};

} /* namespace free_gait_rviz_plugin */
