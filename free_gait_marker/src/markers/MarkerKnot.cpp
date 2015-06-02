/*
 * MarkerKnot.cpp
 *
 *  Created on: Mar 18, 2015
 *      Author: Dario Bellicoso
 */

#include "free_gait_marker/markers/MarkerKnot.hpp"

namespace free_gait_marker {
namespace markers {

MarkerKnot::MarkerKnot() :
    markerScale_(0),
    markerRadius_(0)
{
  // TODO Auto-generated constructor stub

}

MarkerKnot::~MarkerKnot()
{
  // TODO Auto-generated destructor stub
}

void MarkerKnot::loadParameters(
    ros::NodeHandle& nodeHandle)
{
  nodeHandle.param("knot/scale", markerScale_, 0.075);
  nodeHandle.param("knot/radius", markerRadius_, 0.03);

  double color;
  nodeHandle.param("knot/color/r", color, 1.0);
  markerColor_.r = (float) color;
  nodeHandle.param("knot/color/g", color, 0.0);
  markerColor_.g = (float) color;
  nodeHandle.param("knot/color/b", color, 0.0);
  markerColor_.b = (float) color;
  nodeHandle.param("knot/color/a", color, 1.0);
  markerColor_.a = (float) color;
}

// Setup the marker.
void MarkerKnot::setupMarker(const unsigned int markerNumber,
                             const std::string& markerName) {
  controls.clear();
  menu_entries.clear();

  header.frame_id = "map";
  name = markerName;
  description = "Knot: " + std::to_string(markerNumber) + " " + markerName;
  scale = markerScale_;

  // Add a control which contains the sphere and the menu.
  visualization_msgs::InteractiveMarkerControl sphereControl;
  sphereControl.name = name + "_menu";
  sphereControl.always_visible = true;
  // Set the control to drag the sphere in space
  sphereControl.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
  sphereControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

  visualization_msgs::Marker sphereMarker;
  sphereMarker.type = visualization_msgs::Marker::SPHERE;
  double radius = markerRadius_;
  sphereMarker.scale.x = radius;
  sphereMarker.scale.y = radius;
  sphereMarker.scale.z = radius;
  sphereMarker.color = markerColor_;
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

//  control.name = "rotate_x";
//  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//  controls.push_back(control);

  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  controls.push_back(control);


  // rotate and move about y axis
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;

//  control.name = "rotate_y";
//  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//  controls.push_back(control);

  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  controls.push_back(control);


  // rotate and move about z axis
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;

//  control.name = "rotate_z";
//  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//  controls.push_back(control);

  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  controls.push_back(control);
}

} /* namespace markers */
} /* namespace free_gait_marker */
