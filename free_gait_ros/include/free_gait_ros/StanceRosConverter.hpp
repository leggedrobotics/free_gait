/*
 * StanceRosConverter.hppt
 *
 *  Created on: Jul 3, 2017
 *      Author: Péter Fankhauser
 */

#pragma once

#include <free_gait_core/TypeDefs.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

namespace free_gait {

class StanceRosConverter
{
 public:
  StanceRosConverter();
  virtual ~StanceRosConverter();

  static void toMarker(const free_gait::Stance& stance, const std::string& frameId, const std_msgs::ColorRGBA& color,
                       visualization_msgs::Marker& marker);
};

}
