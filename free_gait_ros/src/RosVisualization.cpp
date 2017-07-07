/*
 * RosVisualization.cpp
 *
 *  Created on: Jul 4, 2017
 *      Author: Péter Fankhauser
 */

#include <free_gait_ros/RosVisualization.hpp>

#include <grid_map_core/Polygon.hpp>
#include <grid_map_ros/PolygonRosConverter.hpp>
#include <kindr_ros/RosGeometryMsgPose.hpp>

namespace free_gait {

RosVisualization::RosVisualization()
{
}

RosVisualization::~RosVisualization()
{
}

const visualization_msgs::Marker RosVisualization::getStanceMarker(const Stance& stance, const std::string& frameId,
                                                                   const std_msgs::ColorRGBA& color)
{
  grid_map::Polygon polygon;
  polygon.setFrameId(frameId);
  std::vector<Position> footholdsOrdered;
  getFootholdsCounterClockwiseOrdered(stance, footholdsOrdered);
  double height = 0.0;
  for (auto foothold : footholdsOrdered) {
    polygon.addVertex(foothold.vector().head<2>());
    height += foothold.z();
  }
  height = height / footholdsOrdered.size();

  visualization_msgs::Marker marker;
  grid_map::PolygonRosConverter::toTriangleListMarker(polygon, color, height, marker);
  marker.ns = "Stance";
  marker.lifetime = ros::Duration(0.0);
  return marker;
}

const visualization_msgs::Marker RosVisualization::getComMarker(const Position& comPosition, const std::string& frameId,
                                                                const std_msgs::ColorRGBA& color,
                                                                const double size)
{
  visualization_msgs::Marker marker;
  marker.ns = "Center of Mass";
  marker.header.frame_id = frameId;
  marker.lifetime = ros::Duration(0.0);
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.color = color;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  Pose pose;
  pose.getPosition() = comPosition;
  kindr_ros::convertToRosGeometryMsg(pose, marker.pose);
  return marker;
}

const visualization_msgs::MarkerArray RosVisualization::getComWithProjectionMarker(const Position& comPosition,
                                                                                   const std::string& frameId,
                                                                                   const std_msgs::ColorRGBA& color,
                                                                                   const double comMarkerSize,
                                                                                   const double projectionLenght,
                                                                                   const double projectionDiameter)
{
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.push_back(getComMarker(comPosition, frameId, color, comMarkerSize));

  visualization_msgs::Marker marker;
  marker.ns = "Center of Mass Projection";
  marker.header.frame_id = frameId;
  marker.lifetime = ros::Duration(0.0);
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.color = color;
  marker.scale.x = projectionDiameter;
  marker.scale.y = projectionDiameter;
  marker.scale.z = projectionLenght;
  Pose pose;
  pose.getPosition() = comPosition;
  kindr_ros::convertToRosGeometryMsg(pose, marker.pose);
  marker.pose.position.z -= 0.5 * projectionLenght;
  markerArray.markers.push_back(marker);

  return markerArray;
}

}
