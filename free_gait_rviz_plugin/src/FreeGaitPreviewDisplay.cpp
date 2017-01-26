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
#include <rviz/properties/button_toggle_property.h>
#include <rviz/properties/float_slider_property.h>
#include <rviz/properties/ros_topic_property.h>

// STD
#include <functional>
#include <stdlib.h>

namespace free_gait_rviz_plugin {

FreeGaitPreviewDisplay::FreeGaitPreviewDisplay()
    : adapterRos_(update_nh_, free_gait::AdapterRos::AdapterType::Preview),
      playback_(update_nh_, adapterRos_.getAdapter()),
      stepRosConverter_(adapterRos_.getAdapter()),
      visual_(NULL),
      robotModelRvizPlugin_(NULL)
{
  srand(time(NULL));
  identifier_ = rand();

  playback_.addNewGoalCallback(std::bind(&FreeGaitPreviewDisplay::newGoalAvailable, this));
  playback_.addStateChangedCallback(
      std::bind(&FreeGaitPreviewDisplay::previewStateChanged, this, std::placeholders::_1));
  playback_.addReachedEndCallback(std::bind(&FreeGaitPreviewDisplay::previewReachedEnd, this));

  settingsTree_ = new Property("Settings", QVariant(), "", this);

  goalTopicProperty_ = new rviz::RosTopicProperty("Goal Topic", "", "", "", settingsTree_,
                                                  SLOT(updateTopic()), this);
  QString goalMessageType = QString::fromStdString(
      ros::message_traits::datatype<free_gait_msgs::ExecuteStepsActionGoal>());
  goalTopicProperty_->setMessageType(goalMessageType);
  goalTopicProperty_->setDescription(goalMessageType + " topic to subscribe to.");

  robotStateTopicProperty_ = new rviz::RosTopicProperty("Robot State Topic", "", "", "",
                                                        settingsTree_, SLOT(updateTopic()), this);
  QString robotStateMessageType = QString::fromStdString(adapterRos_.getRobotStateMessageType());
  robotStateTopicProperty_->setMessageType(robotStateMessageType);
  robotStateTopicProperty_->setDescription(robotStateMessageType + " topic to subscribe to.");

  autoPlayProperty_ = new rviz::BoolProperty("Auto-Play", true, "Play motion once received.",
                                             settingsTree_, SLOT(changeAutoPlay()), this);

  autoEnableVisualsProperty_ = new rviz::BoolProperty(
      "Auto En/Disable Visuals", false,
      "Automatically enable and disable visuals for the duration of the preview.", settingsTree_,
      SLOT(changeAutoEnableVisuals()), this);

  previewRateRoperty_ = new rviz::FloatProperty(
      "Preview Rate", playback_.getRate(), "Rate in Hz at which to simulate the motion preview.",
      settingsTree_, SLOT(changePreviewRate()), this);
  previewRateRoperty_->setMin(0.0);

  playbackTree_ = new Property("Playback", QVariant(), "", this);

  playbackSpeedProperty_ = new rviz::FloatSliderProperty("Playback Speed", 1.0,
                                                         "Playback speed factor.", playbackTree_,
                                                         SLOT(changePlaybackSpeed()), this);
  playbackSpeedProperty_->setMin(0.0);
  playbackSpeedProperty_->setMax(10.0);

  playButtonProperty_ = new rviz::ButtonToggleProperty("Play", false, "Play back the motion.",
                                                       playbackTree_, SLOT(startAndStopPlayback()),
                                                       this);
  playButtonProperty_->setLabels("Play", "Pause");
  playButtonProperty_->setReadOnly(true);

  timelimeSliderProperty_ = new rviz::FloatSliderProperty(
      "Time", 0.0, "Determine the current time to visualize the motion.", playbackTree_,
      SLOT(jumpToTime()), this);
  timelimeSliderProperty_->setReadOnly(true);

  visualsTree_ = new rviz::BoolProperty("Visuals", true, "Show/hide all enabled visuals.", this,
                                        SLOT(changeShowAllVisuals()));
  visualsTree_->setDisableChildrenIfFalse(true);

  showEndEffectorTrajectoriesProperty_ = new rviz::BoolProperty(
      "End Effector Trajectories", true, "Draw a trace for the end effector trajectory.",
      visualsTree_, SLOT(changeShowEndEffectorTrajectories()), this);
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

const unsigned int FreeGaitPreviewDisplay::getIdentifier() const
{
  return identifier_;
}

void FreeGaitPreviewDisplay::updateTopic()
{
  unsubscribe();
  reset();
  subscribe();
}

void FreeGaitPreviewDisplay::changeAutoPlay()
{
  ROS_DEBUG_STREAM("Setting auto-play to " << (autoPlayProperty_->getBool() ? "True" : "False") << ".");
}

void FreeGaitPreviewDisplay::changeAutoEnableVisuals()
{
  ROS_DEBUG_STREAM("Setting auto en/disable visuals to " << (autoEnableVisualsProperty_->getBool() ? "True" : "False") << ".");
  if (autoEnableVisualsProperty_->getBool()) findAssociatedRobotModelPlugin();
}

void FreeGaitPreviewDisplay::changePreviewRate()
{
  ROS_DEBUG_STREAM("Setting preview rate to " << previewRateRoperty_->getFloat() << ".");
  playback_.setRate(previewRateRoperty_->getFloat());
}

void FreeGaitPreviewDisplay::changePlaybackSpeed()
{
  ROS_DEBUG_STREAM("Setting playback speed to " << playbackSpeedProperty_->getFloat() << ".");
  playback_.setSpeedFactor(playbackSpeedProperty_->getFloat());
}

void FreeGaitPreviewDisplay::startAndStopPlayback()
{
  if (playButtonProperty_->getBool()) {
    ROS_DEBUG("Pressed start.");
    playback_.run();
  } else {
    ROS_DEBUG("Pressed stop.");
    playback_.stop();
  }
}

void FreeGaitPreviewDisplay::jumpToTime()
{
  playback_.goToTime(ros::Time(timelimeSliderProperty_->getFloat()));
}

void FreeGaitPreviewDisplay::newGoalAvailable()
{
  ROS_DEBUG("FreeGaitPreviewDisplay::newGoalAvailable: New preview available.");
  // Auto-enable visuals.
  if (autoEnableVisualsProperty_->getBool()) {
    setEnabledRobotModel(true);
    visualsTree_->setBool(true);
  }

  // Visuals.
  ROS_DEBUG("FreeGaitPreviewDisplay::newGoalAvailable: Drawing visualizations.");
  visual_->setStateBatch(playback_.getStateBatch());
  visual_->showEnabled();

  // Play back.
  ROS_DEBUG("FreeGaitPreviewDisplay::newGoalAvailable: Setting up control.");
  playButtonProperty_->setReadOnly(false);
  timelimeSliderProperty_->setValuePassive(playback_.getTime().toSec());
  timelimeSliderProperty_->setMin(playback_.getStateBatch().getStartTime());
  timelimeSliderProperty_->setMax(playback_.getStateBatch().getEndTime());
  ROS_DEBUG_STREAM("Setting slider min and max time to: " << timelimeSliderProperty_->getMin()
                   << " & " << timelimeSliderProperty_->getMax() << ".");
  timelimeSliderProperty_->setReadOnly(false);

  // Play.
  if (autoPlayProperty_->getBool()) {
    ROS_DEBUG("FreeGaitPreviewDisplay::newGoalAvailable: Starting playback.");
    playback_.run();
    playButtonProperty_->setLabel("Pause");
  }
}

void FreeGaitPreviewDisplay::previewStateChanged(const ros::Time& time)
{
  timelimeSliderProperty_->setValuePassive(time.toSec());
}

void FreeGaitPreviewDisplay::previewReachedEnd()
{
  ROS_DEBUG("FreeGaitPreviewDisplay::previewReachedEnd: Reached end of preview.");
  if (autoEnableVisualsProperty_->getBool()) {
    setEnabledRobotModel(false);
    visualsTree_->setBool(false);
  }
}

void FreeGaitPreviewDisplay::changeShowAllVisuals()
{
  ROS_DEBUG_STREAM("Setting show all visuals to " << (visualsTree_->getBool() ? "True" : "False") << ".");
  if (visualsTree_->getBool()) {
    visual_->showEnabled();
  } else {
    visual_->hideEnabled();
  }
}

void FreeGaitPreviewDisplay::changeShowEndEffectorTrajectories()
{
  ROS_DEBUG_STREAM("Setting show end effector trajectories to " << (showEndEffectorTrajectoriesProperty_->getBool() ? "True" : "False") << ".");
  visual_->setEnabledModul(FreeGaitPreviewVisual::Modul::EndEffectorTrajectories, showEndEffectorTrajectoriesProperty_->getBool());
  if (showEndEffectorTrajectoriesProperty_->getBool()) {
    visual_->showEndEffectorTrajectories();
  } else {
    visual_->hideEndEffectorTrajectories();
  }
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
  ROS_DEBUG("FreeGaitPreviewDisplay::processMessage: Starting to process new goal.");
  std::vector<free_gait::Step> steps;
  stepRosConverter_.fromMessage(message->goal.steps, steps);
  adapterRos_.updateAdapterWithState();
  playback_.process(steps);
}

void FreeGaitPreviewDisplay::setEnabledRobotModel(bool enable)
{
  if (robotModelRvizPlugin_ == NULL) return;
  robotModelRvizPlugin_->setEnabled(enable);
}

bool FreeGaitPreviewDisplay::findAssociatedRobotModelPlugin()
{
  ROS_DEBUG("Trying to find associated RobotModel RViz plugin.");
  const int nPlugins = context_->getRootDisplayGroup()->numDisplays();
  ROS_DEBUG("There are " << (std::string) nPlugins << " plugins to check.");
  robotModelRvizPlugin_ = NULL;

  // Find index of of this plugin.
  size_t thisIndex;
  for (size_t i = 0; i < context_->getRootDisplayGroup()->numDisplays(); ++i) {
    Display* display = context_->getRootDisplayGroup()->getDisplayAt(i);
    if (getClassId() != display->getClassId()) continue;
    if (identifier_ != static_cast<FreeGaitPreviewDisplay*>(display)->getIdentifier()) continue;
    thisIndex = i;
  }
  ROS_DEBUG_STREAM("This plugin has index number " << thisIndex << ".");

  // Find next RobotModel plugin.
  for (size_t i = thisIndex; i < context_->getRootDisplayGroup()->numDisplays(); ++i) {
    Display* display = context_->getRootDisplayGroup()->getDisplayAt(i);
    std::cout << display->getClassId().toStdString() << std::endl;
    if (display->getClassId() != "rviz/RobotModel") continue;
    robotModelRvizPlugin_ = display;
    ROS_DEBUG_STREAM("Found RobotModel plugin at index " << i << ".");
    break;
  }

  return (robotModelRvizPlugin_ != NULL);
}

}  // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(free_gait_rviz_plugin::FreeGaitPreviewDisplay, rviz::Display)
