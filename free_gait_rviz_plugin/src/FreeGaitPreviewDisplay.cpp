/*
 * FreeGaitPreviewDisplay.cpp
 *
 *  Created on: Nov 29, 2016
 *  Author: Péter Fankhauser
 *  Institute: ETH Zurich, Robotic Systems Lab
 */

#include <free_gait_rviz_plugin/FreeGaitPreviewGeometryVisual.hpp>
#include "free_gait_rviz_plugin/FreeGaitPreviewDisplay.hpp"
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

// ROS
#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/button_property.h>
#include <rviz/properties/float_slider_property.h>
#include <rviz/properties/ros_topic_property.h>

// STD
#include <functional>

namespace free_gait_rviz_plugin {

FreeGaitPreviewDisplay::FreeGaitPreviewDisplay()
    : nodeHandle_("~/free_gait_rviz_plugin"),
      adapterRos_(nodeHandle_, free_gait::AdapterRos::AdapterType::Base),
      playback_(nodeHandle_, adapterRos_.getAdapter())
{
  goalTopicProperty_ = new rviz::RosTopicProperty("Goal Topic", "", "", "", this, SLOT(updateTopic()));
  QString messageType = QString::fromStdString(ros::message_traits::datatype<free_gait_msgs::ExecuteStepsActionGoal>());
  goalTopicProperty_->setMessageType(messageType);
  goalTopicProperty_->setDescription(messageType + " topic to subscribe to.");

  playback_.addNewGoalCallback(std::bind(&FreeGaitPreviewDisplay::newGoalAvailable, this));
  playback_.addReachedEndCallback(std::bind(&FreeGaitPreviewDisplay::previewReachedEnd, this));

  autoPlayProperty_ = new rviz::BoolProperty("Auto-Play", true, "Play motion once received.", this,
                                            SLOT(updateVisualization()));

  playButtonProperty_ = new rviz::ButtonProperty("Play", "Nothing to preview", "Play back the motion.",
                                                 this, SLOT(startAndStopPlayback()));
//  playButtonProperty_->setReadOnly(true);

  timelimeSliderProperty_ = new rviz::FloatSliderProperty("Scroll", 0.0,
                                                          "Scroll through the Free Gait motion.",
                                                          this, SLOT(jumpToTime()));
  timelimeSliderProperty_->setMin(0.0);
  timelimeSliderProperty_->setMax(5.0);
//  sliderProperty_->setReadOnly(true);
}

FreeGaitPreviewDisplay::~FreeGaitPreviewDisplay()
{
  nodeHandle_.shutdown();
//  unsubscribe();
//  delete tf_filter_;
}

void FreeGaitPreviewDisplay::onInitialize()
{
//  active_(true);
//  MFDClass::onInitialize();	 //  "MFDClass" = typedef of "MessageFilterDisplay<message type>"
}

void FreeGaitPreviewDisplay::reset()
{
//  MFDClass::reset();
//  visuals_.clear();
}

void FreeGaitPreviewDisplay::updateVisualization()
{
  std::cout << "FreeGaitPreviewDisplay::updateVisualization()" << std::endl;
}

void FreeGaitPreviewDisplay::startAndStopPlayback()
{
  playback_.process(free_gait::StepQueue());
}

void FreeGaitPreviewDisplay::jumpToTime()
{
  playback_.stop();
  playback_.goToTime(ros::Time(timelimeSliderProperty_->getFloat()));
}

void FreeGaitPreviewDisplay::newGoalAvailable()
{
  std::cout << "NEW GOAL!" << std::endl;
  timelimeSliderProperty_->setMin(playback_.getStateBatch().getStartTime());
  timelimeSliderProperty_->setMax(playback_.getStateBatch().getEndTime());
  playback_.run();
}

void FreeGaitPreviewDisplay::previewReachedEnd()
{
  std::cout << "REACHE END!" << std::endl;
}

void FreeGaitPreviewDisplay::processMessage(const free_gait_msgs::ExecuteStepsActionGoal::ConstPtr& message)
{
  free_gait::StateRosPublisher stateRosPublisher(nodeHandle_, adapterRos_.getAdapter());
  free_gait::State state;

  ros::Duration duration(0.2);
  for (size_t i = 0; i < 100; ++i) {
    state.setRandom();
    stateRosPublisher.publish(state);
    duration.sleep();
  }
}

}  // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(free_gait_rviz_plugin::FreeGaitPreviewDisplay, rviz::Display)
