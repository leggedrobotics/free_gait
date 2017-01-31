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

namespace free_gait_rviz_plugin {

FreeGaitPreviewDisplay::FreeGaitPreviewDisplay()
    : adapterRos_(update_nh_, free_gait::AdapterRos::AdapterType::Preview),
      playback_(update_nh_, adapterRos_.getAdapter()),
      stepRosConverter_(adapterRos_.getAdapter()),
      visual_(NULL),
      robotModelRvizPlugin_(NULL)
{
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

  tfPrefixProperty_ = new rviz::StringProperty(
      "TF Prefix", "",
      "TF prefix used for the preview. Set the same value in the robot model plugin.",
      settingsTree_, SLOT(changeTfPrefix()), this);

  previewRateRoperty_ = new rviz::FloatProperty(
      "Preview Rate", playback_.getRate(), "Rate in Hz at which to simulate the motion preview.",
      settingsTree_, SLOT(changePreviewRate()), this);
  previewRateRoperty_->setMin(0.0);

  autoPlayProperty_ = new rviz::BoolProperty("Auto-Play", true, "Play motion once received.",
                                             settingsTree_, SLOT(changeAutoPlay()), this);

  autoEnableVisualsProperty_ = new rviz::BoolProperty(
      "Auto En/Disable Visuals", false,
      "Automatically enable and disable visuals for the duration of the preview.", settingsTree_,
      SLOT(changeAutoEnableVisuals()), this);

  robotModelProperty_ = new rviz::EditableEnumProperty(
      "Robot Model", "", "Select the robot model used for preview. Robot model must be in the same RViz group.", settingsTree_,
      SLOT(changeRobotModel()), this);
  robotModelProperty_->hide();

  autoHideVisualsProperty_ = new rviz::EnumProperty(
      "Auto Hide When", "Reached End of Preview", "Select the event when to hide the visuals.",
      settingsTree_, SLOT(changeAutoHideVisuals()), this);
  autoHideVisualsProperty_->addOption("Reached End of Preview", 0);
  autoHideVisualsProperty_->addOption("Received Action Result", 1);
  autoHideVisualsProperty_->hide();

  resultTopicProperty_ = new rviz::RosTopicProperty("Result Topic", "", "", "", settingsTree_,
                                                    SLOT(updateTopic()), this);
  QString resultMessageType = QString::fromStdString(
      ros::message_traits::datatype<free_gait_msgs::ExecuteStepsActionResult>());
  resultTopicProperty_->setMessageType(resultMessageType);
  resultTopicProperty_->setDescription(resultMessageType + " topic to subscribe to.");
  resultTopicProperty_->hide();

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

void FreeGaitPreviewDisplay::updateTopic()
{
  unsubscribe();
  reset();
  subscribe();
}

void FreeGaitPreviewDisplay::changeTfPrefix()
{
  ROS_DEBUG_STREAM("Setting tf prefix to " << tfPrefixProperty_->getStdString() << ".");
  playback_.setTfPrefix(tfPrefixProperty_->getStdString());
}

void FreeGaitPreviewDisplay::changePreviewRate()
{
  ROS_DEBUG_STREAM("Setting preview rate to " << previewRateRoperty_->getFloat() << ".");
  playback_.setRate(previewRateRoperty_->getFloat());
}

void FreeGaitPreviewDisplay::changeAutoPlay()
{
  ROS_DEBUG_STREAM("Setting auto-play to " << (autoPlayProperty_->getBool() ? "True" : "False") << ".");
}

void FreeGaitPreviewDisplay::changeAutoEnableVisuals()
{
  ROS_DEBUG_STREAM("Setting auto en/disable visuals to " << (autoEnableVisualsProperty_->getBool() ? "True" : "False") << ".");
  if (autoEnableVisualsProperty_->getBool()) {
    robotModelProperty_->show();
    autoHideVisualsProperty_->show();
    if (autoHideVisualsProperty_->getOptionInt() == AutoHideMode::ReceivedResult) resultTopicProperty_->show();
    robotModelProperty_->clearOptions();
    for (size_t i = 0; i < getParent()->numChildren(); ++i) {
      rviz::Display* display = dynamic_cast<rviz::Display*>(getParent()->childAt(i));
      if (display == NULL) continue;
      if (display->getClassId() != "rviz/RobotModel") continue;
      robotModelProperty_->addOption(display->getName());
    }
  } else {
    robotModelProperty_->hide();
    autoHideVisualsProperty_->hide();
    resultTopicProperty_->hide();
  }
}

void FreeGaitPreviewDisplay::changeRobotModel()
{
  ROS_DEBUG_STREAM("Setting robot model name to " << robotModelProperty_->getStdString() << ".");
  findAssociatedRobotModelPlugin();
}

void FreeGaitPreviewDisplay::changeAutoHideVisuals()
{
  ROS_DEBUG_STREAM("Changed auto-hide visuals event to " << autoHideVisualsProperty_->getStdString() << ".");
  switch (autoHideVisualsProperty_->getOptionInt()) {
    case AutoHideMode::ReachedPreviewEnd:
      resultTopicProperty_->hide();
      break;
    case AutoHideMode::ReceivedResult:
      resultTopicProperty_->show();
      break;
    default:
      break;
  }
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
  ROS_DEBUG("FreeGaitPreviewDisplay::newGoalAvailable: Drawing visualizations.");
  visual_->setStateBatch(playback_.getStateBatch());
  if (autoEnableVisualsProperty_->getBool()) {
    setEnabledRobotModel(true);
    visualsTree_->setBool(true);
  } else {
    visual_->showEnabled();
  }

  // Playback.
  ROS_DEBUG("FreeGaitPreviewDisplay::newGoalAvailable: Setting up control.");
  playButtonProperty_->setReadOnly(false);
  const double midTime = (playback_.getStateBatch().getEndTime() - playback_.getStateBatch().getStartTime()) / 2.0;
  timelimeSliderProperty_->setValuePassive(midTime); // This is required for not triggering value change signal.
  timelimeSliderProperty_->setMin(playback_.getStateBatch().getStartTime());
  timelimeSliderProperty_->setMax(playback_.getStateBatch().getEndTime());
  timelimeSliderProperty_->setValuePassive(playback_.getTime().toSec());
  timelimeSliderProperty_->setReadOnly(false);
  ROS_DEBUG_STREAM("Setting slider min and max time to: " << timelimeSliderProperty_->getMin()
                   << " & " << timelimeSliderProperty_->getMax() << ".");

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
  if (autoEnableVisualsProperty_->getBool() && (autoHideVisualsProperty_->getOptionInt() == AutoHideMode::ReachedPreviewEnd)) {
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

  if (autoHideVisualsProperty_->getOptionInt() == AutoHideMode::ReceivedResult) {
    try {
      resultSubscriber_ = update_nh_.subscribe(resultTopicProperty_->getTopicStd(), 1, &FreeGaitPreviewDisplay::resultCallback, this);
      setStatus(rviz::StatusProperty::Ok, "Result Topic", "OK");
    } catch (ros::Exception& e) {
      setStatus(rviz::StatusProperty::Error, "Result Topic", QString("Error subscribing: ") + e.what());
    }
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
  resultSubscriber_.shutdown();
  adapterRos_.unsubscribeFromRobotState();
}

void FreeGaitPreviewDisplay::processMessage(const free_gait_msgs::ExecuteStepsActionGoal::ConstPtr& message)
{
  ROS_DEBUG("FreeGaitPreviewDisplay::processMessage: Starting to process new goal.");
  std::vector<free_gait::Step> steps;
  stepRosConverter_.fromMessage(message->goal.steps, steps);
  adapterRos_.updateAdapterWithState();
  playback_.process(steps);
}

void FreeGaitPreviewDisplay::resultCallback(const free_gait_msgs::ExecuteStepsActionResult::ConstPtr& message)
{
  ROS_DEBUG("FreeGaitPreviewDisplay::resultCallback: Received result callback.");
  if (autoEnableVisualsProperty_->getBool() && (autoHideVisualsProperty_->getOptionInt() == AutoHideMode::ReceivedResult)) {
    setEnabledRobotModel(false);
    visualsTree_->setBool(false);
  }
}

void FreeGaitPreviewDisplay::setEnabledRobotModel(bool enable)
{
  if (robotModelRvizPlugin_ == NULL) return;
  ROS_DEBUG_STREAM("Setting robot model plugin to enabled " << (enable ? "True" : "False") << ".");
  robotModelRvizPlugin_->setEnabled(enable);
}

bool FreeGaitPreviewDisplay::findAssociatedRobotModelPlugin()
{
  const QString name = robotModelProperty_->getString();
  ROS_DEBUG_STREAM("Trying to find RobotModel RViz plugin with name " << name.toStdString() << ".");
  auto property = getParent()->subProp(name);
  robotModelRvizPlugin_ = dynamic_cast<rviz::Display*>(property);
  return (robotModelRvizPlugin_ != NULL);
}

}  // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(free_gait_rviz_plugin::FreeGaitPreviewDisplay, rviz::Display)
