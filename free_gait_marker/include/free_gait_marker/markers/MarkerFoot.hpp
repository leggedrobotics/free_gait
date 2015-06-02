/*
 * MarkerFoot.hpp
 *
 *  Created on: Mar 18, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarker.h>

namespace free_gait_marker {
namespace markers {

class MarkerFoot : public visualization_msgs::InteractiveMarker
{
 public:
  MarkerFoot();
  virtual ~MarkerFoot();

  // Setup the marker.
  void setupFootholdMarker(const unsigned int stepNumber,
                           const std::string& legName);

  // Load marker parameters
  void loadParameters(
      ros::NodeHandle& nodeHandle);

 private:
  //! Foothold frame id.
  std::string footholdFrameId_;

  //! Foothold marker scale.
  double footholdScale_;

  //! Foothold marker radius.
  double footholdRadius_;

  //! Foothold marker color.
  std_msgs::ColorRGBA footholdColor_;

};

} /* namespace markers */
} /* namespace free_gait_marker */

