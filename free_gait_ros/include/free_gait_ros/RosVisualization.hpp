/*
 * RosVisualization.hpp
 *
 *  Created on: Jul 4, 2017
 *      Author: Péter Fankhauser
 */

#pragma once

#include <free_gait_core/free_gait_core.hpp>

#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

namespace free_gait {

class RosVisualization
{
 public:
  RosVisualization();
  virtual ~RosVisualization();

  static const visualization_msgs::Marker getStanceMarker(const Stance& stance, const std::string& frameId,
                                                          const std_msgs::ColorRGBA& color);
  static const visualization_msgs::Marker getComMarker(const Position& comPosition, const std::string& frameId,
                                                       const std_msgs::ColorRGBA& color, const double size);
};

}
