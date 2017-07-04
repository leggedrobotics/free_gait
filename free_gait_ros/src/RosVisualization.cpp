/*
 * RosVisualization.cpp
 *
 *  Created on: Jul 4, 2017
 *      Author: Péter Fankhauser
 */

#include <free_gait_ros/RosVisualization.hpp>

#include <kindr_ros/RosGeometryMsgPose.hpp>

namespace free_gait {

RosVisualization::RosVisualization(const AdapterBase& adapter)
    : adapter_(adapter)
{
}

RosVisualization::~RosVisualization()
{
}

const visualization_msgs::Marker RosVisualization::getStanceMarker(const std_msgs::ColorRGBA& color) const
{
  visualization_msgs::Marker marker;
  return marker;
}

const visualization_msgs::Marker RosVisualization::getComMarker(const std_msgs::ColorRGBA& color, const double size) const
{
  const Position& comPosition = adapter_.getCenterOfMassInWorldFrame();
  visualization_msgs::Marker marker;
//  marker.header.stamp.fromNSec(polygon.getTimestamp());
  marker.ns = "Center of Mass";
  marker.header.frame_id = adapter_.getWorldFrameId();
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

}
