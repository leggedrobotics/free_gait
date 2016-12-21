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

 protected:
  virtual void onInitialize();
  virtual void reset();

 private Q_SLOTS:
  void updateVisualization();
  void startAndStopPlayback();
  void jumpToTime();
  void newGoalAvailable();
  void previewReachedEnd();

 private:
  // Callback for incoming ROS messages
  void processMessage(const free_gait_msgs::ExecuteStepsActionGoal::ConstPtr& message);
  void runPlayback();

  ros::NodeHandle nodeHandle_;
  free_gait::AdapterRos adapterRos_;

  // Circular buffer for visuals
//  boost::circular_buffer<boost::shared_ptr<FreeGaitPreviewVisual> > visuals_;
  FreeGaitPreviewPlayback playback_;

  // Property variables
  rviz::RosTopicProperty* goalTopicProperty_;
  rviz::ButtonProperty* playButtonProperty_;
  rviz::FloatSliderProperty* timelimeSliderProperty_;
  rviz::BoolProperty* autoPlayProperty_;
};

}  // end namespace grid_map_rviz_plugin
