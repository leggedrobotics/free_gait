/*
 * FreeGaitPanel.h
 *
 *  Created on: Nov 29, 2016
 *  Author: Péter Fankhauser
 *  Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#ifndef Q_MOC_RUN
#include "free_gait_rviz_plugin/FreeGaitPreviewPlayback.hpp"
#include "free_gait_rviz_plugin/FreeGaitPreviewVisual.hpp"

#include <free_gait_msgs/ExecuteStepsActionGoal.h>
#include <free_gait_ros/free_gait_ros.hpp>

#include <rviz/message_filter_display.h>
#include <rviz/display_group.h>
#endif

namespace Ogre {
class SceneNode;
}

namespace rviz {
class BoolProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class EditableEnumProperty;
class ButtonToggleProperty;
class FloatSliderProperty;
class RosTopicProperty;
class DisplayGroup;
}

namespace free_gait_rviz_plugin {

class FreeGaitPreviewVisual;
class FreeGaitPreviewDisplay : public rviz::Display
{
Q_OBJECT
 public:
  // Constructor
  FreeGaitPreviewDisplay();
  // Destructor
  virtual ~FreeGaitPreviewDisplay();

  void setTopic(const QString &topic, const QString &datatype);
  void update(float wall_dt, float ros_dt);
  void reset();
  const unsigned int getIdentifier() const;

 protected:
  void onInitialize();
  void onEnable();
  void onDisable();

 private Q_SLOTS:
  void updateTopic();
  void changeAutoPlay();
  void changeAutoEnableVisuals();
  void changePreviewRate();
  void changePlaybackSpeed();
  void startAndStopPlayback();
  void jumpToTime();
  void newGoalAvailable();
  void previewStateChanged(const ros::Time& time);
  void previewReachedEnd();
  void changeShowAllVisuals();
  void changeShowEndEffectorTrajectories();

 private:
  void subscribe();
  void unsubscribe();
  void processMessage(const free_gait_msgs::ExecuteStepsActionGoal::ConstPtr& message);
  void setEnabledRobotModel(bool enable);
  bool findAssociatedRobotModelPlugin();

  free_gait::AdapterRos adapterRos_;

  FreeGaitPreviewPlayback playback_;
  FreeGaitPreviewVisual* visual_;
  free_gait::StepRosConverter stepRosConverter_;
  ros::Subscriber goalSubscriber_;

  // Property variables
  rviz::Property* settingsTree_;
  rviz::RosTopicProperty* goalTopicProperty_;
  rviz::RosTopicProperty* robotStateTopicProperty_;
  rviz::BoolProperty* autoPlayProperty_;
  rviz::BoolProperty* autoEnableVisualsProperty_;
  rviz::FloatProperty* previewRateRoperty_;
  rviz::Property* playbackTree_;
  rviz::FloatSliderProperty* playbackSpeedProperty_;
  rviz::ButtonToggleProperty* playButtonProperty_;
  rviz::FloatSliderProperty* timelimeSliderProperty_;
  rviz::BoolProperty* visualsTree_;
  rviz::BoolProperty* showEndEffectorTrajectoriesProperty_;

  unsigned int identifier_;
  Display* robotModelRvizPlugin_;
};

}  // end namespace grid_map_rviz_plugin
