/*
 * FreeGaitPreviewDisplay.cpp
 *
 *  Created on: Nov 29, 2016
 *  Author: Péter Fankhauser
 *  Institute: ETH Zurich, Robotic Systems Lab
 */

#include "free_gait_rviz_plugin/FreeGaitPreviewDisplay.hpp"

// OGRE
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
    : adapterRos_(update_nh_, free_gait::AdapterRos::AdapterType::Preview),
      playback_(update_nh_, adapterRos_.getAdapter()),
      stepRosConverter_(adapterRos_.getAdapter())
{
  topicsTree_ = new Property( "Topics", QVariant(), "", this);

  goalTopicProperty_ = new rviz::RosTopicProperty("Goal Topic", "", "", "", topicsTree_, SLOT(updateTopic()), this);
  QString goalMessageType = QString::fromStdString(ros::message_traits::datatype<free_gait_msgs::ExecuteStepsActionGoal>());
  goalTopicProperty_->setMessageType(goalMessageType);
  goalTopicProperty_->setDescription(goalMessageType + " topic to subscribe to.");

  robotStateTopicProperty_ = new rviz::RosTopicProperty("Robot State Topic", "", "", "", topicsTree_, SLOT(updateTopic()), this);
  QString robotStateMessageType = QString::fromStdString(adapterRos_.getRobotStateMessageType());
  robotStateTopicProperty_->setMessageType(robotStateMessageType);
  robotStateTopicProperty_->setDescription(robotStateMessageType + " topic to subscribe to.");

  playback_.addNewGoalCallback(std::bind(&FreeGaitPreviewDisplay::newGoalAvailable, this));
  playback_.addStateChangedCallback(std::bind(&FreeGaitPreviewDisplay::previewStateChanged, this, std::placeholders::_1));
  playback_.addReachedEndCallback(std::bind(&FreeGaitPreviewDisplay::previewReachedEnd, this));

  autoPlayProperty_ = new rviz::BoolProperty("Auto-Play", true, "Play motion once received.", this,
                                            SLOT(updateVisualization()));

  playbackSpeedProperty_ = new rviz::FloatSliderProperty("Playback Speed", 1.0,
                                                         "Playback speed factor.", this,
                                                         SLOT(changePlaybackSpeed()));
  playbackSpeedProperty_->setMin(0.0);
  playbackSpeedProperty_->setMax(10.0);

  playButtonProperty_ = new rviz::ButtonProperty("Play", "", "Play back the motion.",
                                                 this, SLOT(startAndStopPlayback()));
//  playButtonProperty_->setReadOnly(true);

  timelimeSliderProperty_ = new rviz::FloatSliderProperty(
      "Time", 0.0, "Determine the current time to visualize the motion.", this, SLOT(jumpToTime()));
  timelimeSliderProperty_->setReadOnly(true);

  previewRateRoperty_ = new rviz::FloatProperty("Preview Rate", 1000.0,
                                          "Rate in Hz at which to simulate the motion preview.",
                                          this, SLOT(changePreviewRate()));
  previewRateRoperty_->setMin(0.0);
}

FreeGaitPreviewDisplay::~FreeGaitPreviewDisplay()
{
  unsubscribe();
}

void FreeGaitPreviewDisplay::setTopic(const QString &topic, const QString &datatype)
{
  goalTopicProperty_->setString(topic);
}

void FreeGaitPreviewDisplay::update(float wall_dt, float ros_dt)
{
  playback_.update(wall_dt);
}

void FreeGaitPreviewDisplay::onInitialize()
{
  visual_ = new FreeGaitPreviewVisual(context_->getSceneManager(), scene_node_);
}

void FreeGaitPreviewDisplay::onEnable()
{
  subscribe();
}

void FreeGaitPreviewDisplay::onDisable()
{
  unsubscribe();
//reset();
}

void FreeGaitPreviewDisplay::reset()
{
//  MFDClass::reset();
//  visuals_.clear();
//  Display::reset();
//  tf_filter_->clear();
//  messages_received_ = 0;
}

void FreeGaitPreviewDisplay::updateTopic()
{
  unsubscribe();
  reset();
  subscribe();
}

void FreeGaitPreviewDisplay::updateVisualization()
{
  std::cout << "FreeGaitPreviewDisplay::updateVisualization()" << std::endl;
}

void FreeGaitPreviewDisplay::startAndStopPlayback()
{
//  playback_.process(free_gait::StepQueue());
}

void FreeGaitPreviewDisplay::jumpToTime()
{
  playback_.stop();
  playback_.goToTime(ros::Time(timelimeSliderProperty_->getFloat()));
}

void FreeGaitPreviewDisplay::newGoalAvailable()
{
  // Play back.
  playButtonProperty_->setReadOnly(false);
  timelimeSliderProperty_->setMin(playback_.getStateBatch().getStartTime());
  timelimeSliderProperty_->setMax(playback_.getStateBatch().getEndTime());
  timelimeSliderProperty_->setReadOnly(false);
  playback_.run();

  // Visuals.
}

void FreeGaitPreviewDisplay::previewStateChanged(const ros::Time& time)
{
  timelimeSliderProperty_->setValuePassive(time.toSec());
}

void FreeGaitPreviewDisplay::previewReachedEnd()
{
}

void FreeGaitPreviewDisplay::subscribe()
{
  if (!isEnabled()) {
    return;
  }

  try {
    goalSubscriber_ = update_nh_.subscribe(goalTopicProperty_->getTopicStd(), 1, &FreeGaitPreviewDisplay::processMessage, this);
    setStatus(rviz::StatusProperty::Ok, "Goal Topic", "OK");
  } catch (ros::Exception& e) {
    setStatus(rviz::StatusProperty::Error, "Goal Topic", QString("Error subscribing: ") + e.what());
  }

  try {
    adapterRos_.subscribeToRobotState(robotStateTopicProperty_->getStdString());
    setStatus(rviz::StatusProperty::Ok, "Robot State Topic", "OK");
  } catch (ros::Exception& e) {
    setStatus(rviz::StatusProperty::Error, "Robot State Topic", QString("Error subscribing: ") + e.what());
  }
}

void FreeGaitPreviewDisplay::unsubscribe()
{
  goalSubscriber_.shutdown();
}

void FreeGaitPreviewDisplay::processMessage(const free_gait_msgs::ExecuteStepsActionGoal::ConstPtr& message)
{
  std::vector<free_gait::Step> steps;
  stepRosConverter_.fromMessage(message->goal.steps, steps);
  adapterRos_.updateAdapterWithState();
  playback_.process(steps);
}

}  // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(free_gait_rviz_plugin::FreeGaitPreviewDisplay, rviz::Display)
