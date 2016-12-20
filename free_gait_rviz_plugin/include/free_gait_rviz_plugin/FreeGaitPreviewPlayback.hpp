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

namespace free_gait_rviz_plugin {

class FreeGaitPreviewPlayback
{
 public:
  enum class PlayMode {
    PAUSED,
    FORWARD,
    BACKWARD
  };

  FreeGaitPreviewPlayback(ros::NodeHandle& nodeHandle,
                          std::shared_ptr<free_gait::AdapterBase> adapter);
  virtual ~FreeGaitPreviewPlayback();

  void addNewGoalCallback(std::function<void()> callback);
  void addReachedEndCallback(std::function<void()> callback);

  bool process(const free_gait::StepQueue& queue);
  bool isProcessing();
  void cancelProcessing();

  void run();
  void stop();
  void goToTime(const double time);
  void clear();

 private:
  void processingCallback(bool success);
  void update();
  void publish(const ros::Time& time);

  ros::NodeHandle nodeHandle_;
  std::thread updateThread_;
  std::function<void()> newGoalCallback_;
  std::function<void()> reachedEndCallback_;
  std::shared_ptr<free_gait::AdapterBase> adapter_;
  std::unique_ptr<free_gait::BatchExecutor> batchExecutor_;
  std::shared_ptr<free_gait::State> executorState_;
  free_gait::StateBatch stateBatch_;
  free_gait::StateRosPublisher stateRosPublisher_;
  PlayMode playMode_;
  ros::Time time_;
};

} /* namespace free_gait_rviz_plugin */
