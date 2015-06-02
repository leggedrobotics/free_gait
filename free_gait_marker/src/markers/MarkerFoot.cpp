/*
 * MarkerFoot.cpp
 *
 *  Created on: Mar 18, 2015
 *      Author: Dario Bellicoso
 */

#include "free_gait_marker/markers/MarkerFoot.hpp"

namespace free_gait_marker {
namespace markers {

MarkerFoot::MarkerFoot() :
    footholdScale_(0),
    footholdRadius_(0)
{
  // TODO Auto-generated constructor stub
}

MarkerFoot::~MarkerFoot()
{
  // TODO Auto-generated destructor stub
}


void MarkerFoot::loadParameters(
    ros::NodeHandle& nodeHandle)
{
  nodeHandle.param("foothold/scale", footholdScale_, 0.075);
  nodeHandle.param("foothold/radius", footholdRadius_, 0.07);
  double color;
  nodeHandle.param("foothold/color/r", color, 0.0);
  footholdColor_.r = (float) color;
  nodeHandle.param("foothold/color/g", color, 0.0);
  footholdColor_.g = (float) color;
  nodeHandle.param("foothold/color/b", color, 0.65);
  footholdColor_.b = (float) color;
  nodeHandle.param("foothold/color/a", color, 1.0);
  footholdColor_.a = (float) color;
}


void MarkerFoot::setupFootholdMarker(const unsigned int stepNumber,
                                     const std::string& legName)
{
  controls.clear();
  menu_entries.clear();

  header.frame_id = footholdFrameId_;
//  name = "foothold_" + std::to_string(stepNumber) + "_" + legName;
  name = legName;
  description = "Foothold " + std::to_string(stepNumber) + " " + legName;
  scale = footholdScale_;

  // Add a control which contains the sphere and the menu.
  visualization_msgs::InteractiveMarkerControl sphereControl;
  sphereControl.name = name + "_menu";
  sphereControl.always_visible = true;
  // Set the control to drag the sphere in space
  sphereControl.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
  sphereControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

  visualization_msgs::Marker sphereMarker;
  sphereMarker.type = visualization_msgs::Marker::SPHERE;
  double radius = footholdRadius_;
  sphereMarker.scale.x = radius;
  sphereMarker.scale.y = radius;
  sphereMarker.scale.z = radius;
  sphereMarker.color = footholdColor_;
  sphereControl.markers.push_back(sphereMarker);

  controls.push_back(sphereControl);

  // Add interactive controls.
  visualization_msgs::InteractiveMarkerControl control;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

  // rotate and move about x axis
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;

  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  controls.push_back(control);

  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  controls.push_back(control);


  // rotate and move about y axis
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;

  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  controls.push_back(control);

  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  controls.push_back(control);


  // rotate and move about z axis
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;

  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  controls.push_back(control);

  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  controls.push_back(control);
}


} /* namespace markers */
} /* namespace free_gait_marker */
