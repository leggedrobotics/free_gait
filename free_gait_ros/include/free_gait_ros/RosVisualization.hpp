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
  RosVisualization(const AdapterBase& adapter);
  virtual ~RosVisualization();

  const visualization_msgs::Marker getStanceMarker(const std_msgs::ColorRGBA& color) const;
  const visualization_msgs::Marker getComMarker(const std_msgs::ColorRGBA& color, const double size) const;

 private:
  const AdapterBase& adapter_;
};

}
