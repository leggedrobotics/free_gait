/*
 * StanceRosConverter.cpp
 *
 *  Created on: Jul 3, 2017
 *      Author: Péter Fankhauser
 */

#include "free_gait_ros/StanceRosConverter.hpp"

#include <ros/ros.h>

namespace free_gait {

StanceRosConverter::StanceRosConverter()
{
}

StanceRosConverter::~StanceRosConverter()
{
}

void StanceRosConverter::toMarker(const free_gait::Stance& stance, const std::string& frameId,
                                  const std_msgs::ColorRGBA& color, visualization_msgs::Marker& marker)
{
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frameId;
  marker.lifetime = ros::Duration(0.0);
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.color = color;

  size_t nTriangles = stance.size() - 2;
  size_t nPoints = 3 * nTriangles;
  if (nTriangles < 1) return;
  marker.points.resize(nPoints);
  marker.colors.resize(nPoints, color);

  std::vector<Position> footholds;
  getFootholdsCounterClockwiseOrdered(stance, footholds);

  for (size_t i = 0; i < nTriangles; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      const size_t footholdIndex = i + j;
      const size_t pointIndex = 3 * i + j;
      marker.points[pointIndex].x = footholds[footholdIndex].x();
      marker.points[pointIndex].y = footholds[footholdIndex].y();
      marker.points[pointIndex].z = footholds[footholdIndex].z();
    }
  }
}

}
