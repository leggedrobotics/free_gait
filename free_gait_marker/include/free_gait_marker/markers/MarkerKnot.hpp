/*
 * MarkerKnot.hpp
 *
 *  Created on: Mar 18, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

#include <visualization_msgs/InteractiveMarker.h>
#include <ros/ros.h>

namespace free_gait_marker {
namespace markers {

class MarkerKnot : public visualization_msgs::InteractiveMarker
{
 public:
  MarkerKnot();
  virtual ~MarkerKnot();

  // Setup the marker.
  void setupMarker(const unsigned int markerNumber,
                   const std::string& markerName);

  // Load marker parameters
  void loadParameters(
      ros::NodeHandle& nodeHandle);

 private:
  //! Foothold marker scale.
  double markerScale_;

  //! Foothold marker radius.
  double markerRadius_;

  //! Foothold marker color.
  std_msgs::ColorRGBA markerColor_;

  //! Timeout duration for the connection to the action server.
  ros::Duration waitForActionTimeout_;
};

} /* namespace markers */
} /* namespace free_gait_marker */
