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

// STD
#include <functional>

namespace free_gait_rviz_plugin {

FreeGaitPreviewDisplay::FreeGaitPreviewDisplay()
    : nodeHandle_("~"),
      adapterRos_(nodeHandle_, free_gait::AdapterRos::AdapterType::Base),
      playback_(nodeHandle_, adapterRos_.getAdapter())
{
  playback_.addNewGoalCallback(std::bind(&FreeGaitPreviewDisplay::newGoalAvailable, this));
  playback_.addReachedEndCallback(std::bind(&FreeGaitPreviewDisplay::previewReachedEnd, this));

  autoPlayProperty_ = new rviz::BoolProperty("Auto-Play", true, "Play motion once received.", this,
                                            SLOT(updateVisualization()));

  playButtonProperty_ = new rviz::ButtonProperty("Play", "Nothing to preview", "Play back the motion.",
                                                 this, SLOT(startAndStopPlayback()));
//  playButtonProperty_->setReadOnly(true);

  sliderProperty_ = new rviz::FloatSliderProperty("Scroll", 0.0,
                                                  "Scroll through the Free Gait motion.", this,
                                                  SLOT(updateVisualization()));
  sliderProperty_->setMin(0.0);
  sliderProperty_->setMax(5.0);
//  sliderProperty_->setReadOnly(true);

}

FreeGaitPreviewDisplay::~FreeGaitPreviewDisplay()
{
}

void FreeGaitPreviewDisplay::onInitialize()
{
  MFDClass::onInitialize();	 //  "MFDClass" = typedef of "MessageFilterDisplay<message type>"
}

void FreeGaitPreviewDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void FreeGaitPreviewDisplay::updateVisualization()
{
  std::cout << "FreeGaitPreviewDisplay::updateVisualization()" << std::endl;
//  float alpha = alphaProperty_->getFloat();
//  bool showGridLines = showGridLinesProperty_->getBool();
//  bool flatTerrain = heightModeProperty_->getOptionInt() == 1;
//  std::string heightLayer = heightTransformerProperty_->getStdString();
//  bool mapLayerColor = colorModeProperty_->getOptionInt() == 1;
//  bool flatColor = colorModeProperty_->getOptionInt() == 2;
//  Ogre::ColourValue meshColor = colorProperty_->getOgreColor();
//  std::string colorLayer = colorTransformerProperty_->getStdString();
//  bool useRainbow = useRainbowProperty_->getBool();
//  bool invertRainbow = invertRainbowProperty_->getBool();
//  Ogre::ColourValue minColor = minColorProperty_->getOgreColor();
//  Ogre::ColourValue maxColor = maxColorProperty_->getOgreColor();
//  bool autocomputeIntensity = autocomputeIntensityBoundsProperty_->getBool();
//  float minIntensity = minIntensityProperty_->getFloat();
//  float maxIntensity = maxIntensityProperty_->getFloat();
//
//  for (size_t i = 0; i < visuals_.size(); i++) {
//    visuals_[i]->computeVisualization(alpha, showGridLines, flatTerrain, heightLayer, flatColor,
//                                      meshColor, mapLayerColor, colorLayer, useRainbow, invertRainbow,
//				      minColor, maxColor, autocomputeIntensity, minIntensity, maxIntensity);
//  }
}

void FreeGaitPreviewDisplay::startAndStopPlayback()
{
  playback_.process(free_gait::StepQueue());
}

void FreeGaitPreviewDisplay::newGoalAvailable()
{
  std::cout << "NEW GOAL!" << std::endl;
  playback_.run();
}

void FreeGaitPreviewDisplay::previewReachedEnd()
{

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

//  // Check if transform between the message's frame and the fixed frame exists.
//  Ogre::Quaternion orientation;
//  Ogre::Vector3 position;
//  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->info.header.stamp,
//                                                 position, orientation)) {
//    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->info.header.frame_id.c_str(),
//              qPrintable(fixed_frame_));
//    return;
//  }
//
//  boost::shared_ptr<GridMapVisual> visual;
//  if (visuals_.full()) {
//    visual = visuals_.front();
//  } else {
//    visual.reset(new GridMapVisual(context_->getSceneManager(), scene_node_));
//  }
//
//  visual->setMessage(msg);
//  visual->setFramePosition(position);
//  visual->setFrameOrientation(orientation);
//
//  visual->computeVisualization(alphaProperty_->getFloat(), showGridLinesProperty_->getBool(),
//                               heightModeProperty_->getOptionInt() == 1,
//                               heightTransformerProperty_->getStdString(),
//                               colorModeProperty_->getOptionInt() == 2,
//                               colorProperty_->getOgreColor(),
//			       colorModeProperty_->getOptionInt() == 1,
//                               colorTransformerProperty_->getStdString(),
//                               useRainbowProperty_->getBool(),
//			       invertRainbowProperty_->getBool(),
//                               minColorProperty_->getOgreColor(),
//                               maxColorProperty_->getOgreColor(),
//                               autocomputeIntensityBoundsProperty_->getBool(),
//                               minIntensityProperty_->getFloat(),
//                               maxIntensityProperty_->getFloat());
//
//  std::vector<std::string> layer_names = visual->getLayerNames();
//  heightTransformerProperty_->clearOptions();
//  colorTransformerProperty_->clearOptions();
//  for (size_t i = 0; i < layer_names.size(); i++) {
//    heightTransformerProperty_->addOptionStd(layer_names[i]);
//    colorTransformerProperty_->addOptionStd(layer_names[i]);
//  }
//
//  visuals_.push_back(visual);
}

}  // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(free_gait_rviz_plugin::FreeGaitPreviewDisplay, rviz::Display)
