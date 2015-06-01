/*
 * FreeGaitMarker.cpp
 *
 *  Created on: Feb 28, 2015
 *      Author: Péter Fankhauser, Dario Bellicoso
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_marker/FreeGaitMarker.hpp"

// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <starleth_msgs/Step.h>

// STD
#include <algorithm>
#include <stdexcept>

using namespace interactive_markers;
using namespace visualization_msgs;

static const int NUMBER_OF_LEGS = 4;
static const std::string LEFT_FORE_NAME = "leftFore";
static const int LEFT_FORE_ID = 0;
static const std::string RIGHT_FORE_NAME = "rightFore";
static const int RIGHT_FORE_ID = 1;
static const std::string LEFT_HIND_NAME = "leftHind";
static const int LEFT_HIND_ID = 2;
static const std::string RIGHT_HIND_NAME = "rightHind";
static const int RIGHT_HIND_ID = 3;

namespace free_gait_marker {

FreeGaitMarker::FreeGaitMarker(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      server_(ros::this_node::getName()),
      knots_(NUMBER_OF_LEGS),
      knotIds_(NUMBER_OF_LEGS, 0),
      trajectoryIds_(NUMBER_OF_LEGS, 0),
      splines_(NUMBER_OF_LEGS),
      trajectories_(NUMBER_OF_LEGS)
{
  loadParameters();

  stepActionClient_ = std::unique_ptr<actionlib::SimpleActionClient<starleth_msgs::StepAction>>(
      new actionlib::SimpleActionClient<starleth_msgs::StepAction>(actionServerTopic_, true));

  // Setup context menu
  interactive_markers::MenuHandler::EntryHandle stepHandle = menuHandler_.insert("Step", boost::bind(&FreeGaitMarker::menuStepCallback, this, _1));
  interactive_markers::MenuHandler::EntryHandle trajHandle = menuHandler_.insert("Trajectory", boost::bind(&FreeGaitMarker::menuTrajectoryCallback, this, _1));

  menuHandler_.insert(stepHandle, "Send Step Goal", boost::bind(&FreeGaitMarker::menuStepCallback, this, _1));
  menuHandler_.insert(stepHandle, "Activate", boost::bind(&FreeGaitMarker::menuStepCallback, this, _1));
  menuHandler_.insert(stepHandle, "Deactivate", boost::bind(&FreeGaitMarker::menuStepCallback, this, _1));

  menuHandler_.insert(trajHandle, "Set knot point", boost::bind(&FreeGaitMarker::menuTrajectoryCallback, this, _1));
  menuHandler_.insert(trajHandle, "Send Trajectory", boost::bind(&FreeGaitMarker::menuTrajectoryCallback, this, _1));
  menuHandler_.insert(trajHandle, "Clear Trajectory", boost::bind(&FreeGaitMarker::menuTrajectoryCallback, this, _1));
  menuHandler_.insert(trajHandle, "Preview", boost::bind(&FreeGaitMarker::menuTrajectoryCallback, this, _1));

  knotsPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("trajectory_knots", 100);
  trajectoriesPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("trajectory_points", 100);

  addFoothold(LEFT_FORE_ID, LEFT_FORE_NAME);
  addFoothold(RIGHT_FORE_ID, RIGHT_FORE_NAME);
  addFoothold(LEFT_HIND_ID, LEFT_HIND_NAME);
  addFoothold(RIGHT_HIND_ID, RIGHT_HIND_NAME);
  applyChanges();
  for (auto& foothold : footholdList_) deactivateFoothold(foothold);
  applyChanges();
}

FreeGaitMarker::~FreeGaitMarker()
{
}

void FreeGaitMarker::loadParameters()
{
  nodeHandle_.param("action_server_topic", actionServerTopic_, std::string("/locomotion_controller/step"));
  double duration;
  nodeHandle_.param("wait_for_action_timeout", duration, 5.0);
  waitForActionTimeout_.fromSec(duration);
  nodeHandle_.param("foothold/frame_id", footholdFrameId_, std::string("map"));
  nodeHandle_.param("foothold/scale", footholdScale_, 0.075);
  nodeHandle_.param("foothold/radius", footholdRadius_, 0.07);
  double color;
  nodeHandle_.param("foothold/color/r", color, 0.0);
  footholdColor_.r = (float) color;
  nodeHandle_.param("foothold/color/g", color, 0.0);
  footholdColor_.g = (float) color;
  nodeHandle_.param("foothold/color/b", color, 0.65);
  footholdColor_.b = (float) color;
  nodeHandle_.param("foothold/color/a", color, 1.0);
  footholdColor_.a = (float) color;
}

void FreeGaitMarker::addFoothold(const unsigned int stepNumber, const std::string& legName)
{
  InteractiveMarker marker;
  setupFootholdMarker(marker, stepNumber, legName);

  std::string footFrameId;
  if (legName == LEFT_FORE_NAME) footFrameId = "LF_FOOT";
  else if (legName == RIGHT_FORE_NAME) footFrameId = "RF_FOOT";
  else if (legName == LEFT_HIND_NAME) footFrameId = "LH_FOOT";
  else if (legName == RIGHT_HIND_NAME) footFrameId = "RH_FOOT";
  else {
    ROS_WARN(("No corresponding frame id for the foot found for leg name `" + legName + "'.").c_str());
    return;
  }

  footholdList_.emplace_back(Foothold{stepNumber, legName, marker.name, footFrameId, true});
  server_.insert(marker, boost::bind(&FreeGaitMarker::footholdCallback, this, _1));
  menuHandler_.apply(server_, marker.name);
}

void FreeGaitMarker::setupFootholdMarker(visualization_msgs::InteractiveMarker& marker,
                                         const unsigned int stepNumber, const std::string& legName)
{
  marker.controls.clear();
  marker.menu_entries.clear();

  marker.header.frame_id = footholdFrameId_;
  marker.name = "foothold_" + std::to_string(stepNumber) + "_" + legName;
  marker.description = "Foothold " + std::to_string(stepNumber) + " " + legName;
  marker.scale = footholdScale_;

  // Add a control which contains the sphere and the menu.
  InteractiveMarkerControl sphereControl;
  sphereControl.name = marker.name + "_menu";
  sphereControl.always_visible = true;
  // Set the control to drag the sphere in space
  sphereControl.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  sphereControl.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;

  Marker sphereMarker;
  sphereMarker.type = Marker::SPHERE;
  double radius = footholdRadius_;
  sphereMarker.scale.x = radius;
  sphereMarker.scale.y = radius;
  sphereMarker.scale.z = radius;
  sphereMarker.color = footholdColor_;
  sphereControl.markers.push_back(sphereMarker);

  marker.controls.push_back(sphereControl);

  // Add interactive controls.
  InteractiveMarkerControl control;
  control.orientation_mode = InteractiveMarkerControl::FIXED;

  // rotate and move about x axis
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;

  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);

  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);


  // rotate and move about y axis
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;

  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);

  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);


  // rotate and move about z axis
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;

  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);

  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

}


void FreeGaitMarker::menuStepCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type != InteractiveMarkerFeedback::MENU_SELECT) {
    ROS_WARN("Feedback not of type 'Menu Select'.");
    return;
  }

  FreeGaitMarker::Foothold foothold;
  switch (feedback->menu_entry_id) {
    case 3: // Send Step Goal
      sendStepGoal();
      break;
    case 4: // Activate
      if (!activateFoothold(getFootholdByMarkerName(feedback->marker_name.c_str())))
        ROS_WARN("Foothold marker with name `%s` could not be activated.",
                 feedback->marker_name.c_str());
      break;
    case 5: // Deactivate
      if (!deactivateFoothold(getFootholdByMarkerName(feedback->marker_name.c_str())))
        ROS_WARN("Foothold marker with name `%s` could not be deactivated.", feedback->marker_name.c_str());
      break;
    default:
      ROS_WARN("Invalid menu entry.");
      return;
  }

  applyChanges();
}


//void FreeGaitMarker::menuCallback(
void FreeGaitMarker::menuTrajectoryCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type != InteractiveMarkerFeedback::MENU_SELECT) {
    ROS_WARN("Feedback not of type 'Menu Select'.");
    return;
  }

  FreeGaitMarker::Foothold foothold;

  switch (feedback->menu_entry_id) {
    // Set knot point
    case 6: {
      // save current marker pose
      const int legId = getFootholdByMarkerName(feedback->marker_name.c_str()).stepNumber;

      if (footholdList_.at(legId).isActive) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "knots_" + footholdList_.at(legId).legName;
        marker.id = knotIds_.at(legId)++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = feedback->pose;
        marker.scale.x = 0.04;
        marker.scale.y = 0.04;
        marker.scale.z = 0.04;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        if (marker.id >= knots_[legId].markers.size()) {
          knots_[legId].markers.push_back(marker);
        } else {
          knots_[legId].markers[marker.id] = marker;
        }
      }

    } break;

    // Send Trajectory
    case 7: {
    } break;

    // Clear Trajectory
    case 8: {
      const int legId = getFootholdByMarkerName(feedback->marker_name.c_str()).stepNumber;
      for (size_t k=0; k<knots_.at(legId).markers.size(); k++) {
        knots_[legId].markers.at(k).action = visualization_msgs::Marker::DELETE;
      }
      for (size_t k=0; k<trajectories_.at(legId).markers.size(); k++) {
        trajectories_[legId].markers.at(k).action = visualization_msgs::Marker::DELETE;
      }

      splines_.at(legId).clearCurve();
      knotIds_.at(legId) = 0;
      trajectoryIds_.at(legId) = 0;
    } break;

    // Preview trajectory
    case 9: {
      const int legId = getFootholdByMarkerName(feedback->marker_name.c_str()).stepNumber;

      std::vector<double> tData;
      std::vector<Eigen::Matrix<double,3,1>> vData;
      double tf = 0.0;

      trajectories_[legId].markers.clear();

      tData.clear();
      vData.clear();
//      for (size_t k=0; k<knots_.at(legId).markers.size(); k++) {
      for (size_t k=0; k<knotIds_[legId]; k++) {
        tData.push_back(tf);
        Eigen::Matrix<double,3,1> vpoint;
        vpoint << knots_[legId].markers.at(k).pose.position.x,
                  knots_[legId].markers.at(k).pose.position.y,
                  knots_[legId].markers.at(k).pose.position.z;
        vData.push_back(vpoint);
        // todo: set tf appropriately
        tf += 1.0;
      }

      splines_.at(legId).fitCurve(tData, vData);

      double dt = 0.05;
      for (int k=0; k*dt < splines_.at(legId).getMaxTime(); k++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "knots_" + footholdList_.at(legId).legName;
        marker.id = trajectoryIds_.at(legId)++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = splines_.at(legId).evaluate(k*dt).x();
        marker.pose.position.y = splines_.at(legId).evaluate(k*dt).y();
        marker.pose.position.z = splines_.at(legId).evaluate(k*dt).z();

        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        trajectories_.at(legId).markers.push_back(marker);
      }

    } break;

    default:
      ROS_WARN("Invalid menu entry.");
      return;
  }

  applyChanges();
}

void FreeGaitMarker::footholdCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
//  Foothold& foothold(getFootholdByMarkerName(feedback->marker_name.c_str()));
//  if (foothold.isActive) return;
//  if (!activateFoodhold(foothold))
//    ROS_WARN("Foothold marker with name `%s` could not be activated.",
//             feedback->marker_name.c_str());
//  applyChanges();
}

bool FreeGaitMarker::activateFoothold(Foothold& foothold)
{
  if (foothold.isActive) return true;

  // Get pose.
  InteractiveMarker marker;
  if (!getMarkerFromFoothold(foothold, marker)) return false;
  tf::Stamped<tf::Point> positionInPassiveFrame; // Foot frame.
  tf::Stamped<tf::Point> positionInActiveFrame; // Map frame.
  tf::pointMsgToTF(marker.pose.position, positionInPassiveFrame);
  positionInPassiveFrame.frame_id_ = foothold.footFrameId;
  positionInPassiveFrame.stamp_ = ros::Time(0.0);
  transformListener_.transformPoint(footholdFrameId_, positionInPassiveFrame, positionInActiveFrame);
  geometry_msgs::Pose pose;
  tf::pointTFToMsg(positionInActiveFrame, pose.position);

  // Get header.
  std_msgs::Header header;
  header.frame_id = positionInActiveFrame.frame_id_;
  header.stamp = positionInActiveFrame.stamp_;

  // Update marker pose.
  if (!server_.setPose(foothold.markerName, pose, header)) {
    ROS_WARN("Marker with name `%s` not found.", foothold.markerName.c_str());
    return false;
  }

  foothold.isActive = true;
  return true;
}

bool FreeGaitMarker::deactivateFoothold(Foothold& foothold)
{
  std_msgs::Header header;
  header.frame_id = foothold.footFrameId;
  header.stamp = ros::Time(0.0);
  if (!server_.setPose(foothold.markerName, geometry_msgs::Pose(), header)) {
    ROS_WARN("Marker with name `%s` not found.", foothold.markerName.c_str());
    return false;
  }

  foothold.isActive = false;
  return true;
}

bool FreeGaitMarker::deactivateAllFootholds()
{
  for (auto& foothold : footholdList_) {
    if (!deactivateFoothold(foothold)) return false;
  }
  return true;
}

void FreeGaitMarker::applyChanges()
{
  server_.applyChanges();
}

FreeGaitMarker::Foothold& FreeGaitMarker::getFootholdByMarkerName(const std::string& markerName)
{
  for (auto& foothold : footholdList_) {
    if (foothold.markerName == markerName) {
      return foothold;
    }
  }
  throw std::out_of_range("Foothold with marker name " + markerName + " not found.");
}

bool FreeGaitMarker::getMarkerFromFoothold(const FreeGaitMarker::Foothold& foothold,
                                           visualization_msgs::InteractiveMarker& marker)
{
  if (!server_.get(foothold.markerName, marker)) {
    ROS_WARN("Marker with name `%s` not found.", foothold.markerName.c_str());
    return false;
  }
  return true;
}

bool FreeGaitMarker::sendStepGoal()
{
  if (!stepActionClient_->isServerConnected()) {
    ROS_INFO("Waiting for step action server to start.");
    if (!stepActionClient_->waitForServer(waitForActionTimeout_)) {
      ROS_WARN("No step action server found, ignoring action.");
      return false;
    }
  }

  std::sort(footholdList_.begin(), footholdList_.end());
  starleth_msgs::StepGoal goal;

  for (const auto& foothold : footholdList_) {
    InteractiveMarker marker;
    if (!getMarkerFromFoothold(foothold, marker)) {
      ROS_WARN("Goal not sent.");
      return false;
    }

    if (!foothold.isActive) continue;
    starleth_msgs::Step step;
    step.step_number = foothold.stepNumber;
    starleth_msgs::SwingData swingData;
    swingData.name = foothold.legName;
    swingData.profile.target.header.frame_id = footholdFrameId_;
    swingData.profile.target.point = marker.pose.position;
    step.swing_data.push_back(swingData);
    goal.steps.push_back(step);
  }

  stepActionClient_->sendGoal(goal);
  stepActionClient_->waitForResult();
  deactivateAllFootholds();
//  actionlib::SimpleClientGoalState state = ac.getState();

  return true;
}


void FreeGaitMarker::publishKnots() {
  for (auto& msg : knots_) {
    knotsPublisher_.publish(msg);
  }

  for (auto& msg : trajectories_) {
    trajectoriesPublisher_.publish(msg);
  }
}


void FreeGaitMarker::print() {
  ROS_INFO_STREAM("Foothold List:" << std::endl);
  for (const auto& foothold : footholdList_) {
    ROS_INFO_STREAM("Step Nr.: " << foothold.stepNumber);
    ROS_INFO_STREAM("Leg Name: " << foothold.legName);
    ROS_INFO_STREAM("Marker Name: " << foothold.markerName);
    ROS_INFO_STREAM("Foot Frame ID: " << foothold.footFrameId);
    ROS_INFO_STREAM("Is Active: " << (foothold.isActive ? "Yes" : "No") << std::endl);
  }
}

} /* namespace free_gait_marker */
