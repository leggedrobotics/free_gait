/*
 * FreeGaitPanel.h
 *
 *  Created on: Nov 29, 2016
 *  Author: Péter Fankhauser
 *  Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#ifndef Q_MOC_RUN
//#include "free_gait_rviz_plugin/properties/buttonProperty.hpp"
#include <boost/circular_buffer.hpp>
#include <rviz/message_filter_display.h>
#include <free_gait_msgs/ExecuteStepsActionGoal.h>
#include <free_gait_ros/free_gait_ros.hpp>
#include <thread>
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
}

namespace free_gait_rviz_plugin {

class FreeGaitPreviewVisual;
class FreeGaitPreviewDisplay : public rviz::MessageFilterDisplay<
    free_gait_msgs::ExecuteStepsActionGoal>
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
  // Qt slots
  void updateHistoryLength();
  void updateHeightMode();
  void updateColorMode();
  void updateUseRainbow();
  void updateAutocomputeIntensityBounds();
  void updateVisualization();

 private:
  // Callback for incoming ROS messages
  void processMessage(const free_gait_msgs::ExecuteStepsActionGoal::ConstPtr& message);
  void runPlayback();

  ros::NodeHandle nodeHandle_;
  free_gait::AdapterRos adapterRos_;
  std::thread playbackThread_;

  // Circular buffer for visuals
  boost::circular_buffer<boost::shared_ptr<FreeGaitPreviewVisual> > visuals_;

  // Property variables
//  rviz::ButtonProperty* playButtonProperty_;
  rviz::FloatProperty* alphaProperty_;
  rviz::IntProperty* historyLengthProperty_;
  rviz::BoolProperty* showGridLinesProperty_;
  rviz::EnumProperty* heightModeProperty_;
  rviz::EditableEnumProperty* heightTransformerProperty_;
  rviz::EnumProperty* colorModeProperty_;
  rviz::EditableEnumProperty* colorTransformerProperty_;
  rviz::ColorProperty* colorProperty_;
  rviz::BoolProperty* useRainbowProperty_;
  rviz::BoolProperty* invertRainbowProperty_;
  rviz::ColorProperty* minColorProperty_;
  rviz::ColorProperty* maxColorProperty_;
  rviz::BoolProperty* autocomputeIntensityBoundsProperty_;
  rviz::FloatProperty* minIntensityProperty_;
  rviz::FloatProperty* maxIntensityProperty_;
};

}  // end namespace grid_map_rviz_plugin
