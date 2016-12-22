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
#include <rviz/message_filter_display.h>
#include <free_gait_msgs/ExecuteStepsActionGoal.h>
#include <free_gait_ros/free_gait_ros.hpp>
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
class ButtonProperty;
class FloatSliderProperty;
class RosTopicProperty;
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

 protected:
  void onInitialize();
  void onEnable();
  void onDisable();

 private Q_SLOTS:
  void updateTopic();
  void updateVisualization();
  void startAndStopPlayback();
  void jumpToTime();
  void newGoalAvailable();
  void previewStateChanged(const ros::Time& time);
  void previewReachedEnd();

 private:
  void subscribe();
  void unsubscribe();
  void processMessage(const free_gait_msgs::ExecuteStepsActionGoal::ConstPtr& message);

  ros::NodeHandle nodeHandle_;
  free_gait::AdapterRos adapterRos_;

//  boost::circular_buffer<boost::shared_ptr<FreeGaitPreviewVisual> > visuals_;
  FreeGaitPreviewPlayback playback_;
  FreeGaitPreviewVisual visual_;
  free_gait::StepRosConverter stepRosConverter_;
  ros::Subscriber goalSubscriber_;

  // Property variables
  rviz::Property* topicsTree_;
  rviz::RosTopicProperty* goalTopicProperty_;
  rviz::RosTopicProperty* robotStateTopicProperty_;
  rviz::BoolProperty* autoPlayProperty_;
  rviz::FloatSliderProperty* playbackSpeedProperty_;
  rviz::ButtonProperty* playButtonProperty_;
  rviz::FloatSliderProperty* timelimeSliderProperty_;
  rviz::FloatProperty* previewRateRoperty_;
};

}  // end namespace grid_map_rviz_plugin
