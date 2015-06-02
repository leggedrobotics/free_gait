/*
 * FreeGaitMarker.hpp
 *
 *  Created on: Feb 28, 2015
 *      Author: Péter Fankhauser, Dario Bellicoso
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <quadruped_msgs/StepAction.h>
#include <tf/transform_listener.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

// STD
#include <string>
#include <vector>

// robot_utils
#include "robotUtils/curves/PolynomialSplineVectorSpaceCurve.hpp"

namespace free_gait_marker {

class FreeGaitMarker
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  FreeGaitMarker(ros::NodeHandle& nodeHandle);

  virtual ~FreeGaitMarker();

  /*!
   * Definition of the container for the step data.
   */
  struct Foothold
  {
    //! Step number.
    unsigned int stepNumber;

    //! Leg name identifier.
    std::string legName;

    //! Name of the interactive marker.
    std::string markerName;

    //! Frame id of the corresponding foot.
    std::string footFrameId;

    //! True if active, false otherwise.
    bool isActive;

    bool operator<(const Foothold& foothold) const
    {
      return stepNumber < foothold.stepNumber;
    }
  };

  void loadParameters();

  //Note: This change will not take effect until you call applyChanges()
  void addFoothold(const unsigned int stepNumber, const std::string& legName);

  //Note: This change will not take effect until you call applyChanges()
  bool activateFoothold(Foothold& foothold);

  //Note: This change will not take effect until you call applyChanges()
  bool deactivateFoothold(Foothold& foothold);

  //Note: This change will not take effect until you call applyChanges()
  bool deactivateAllFootholds();

  void applyChanges();

  FreeGaitMarker::Foothold& getFootholdByMarkerName(const std::string& markerName);

  bool getMarkerFromFoothold(const FreeGaitMarker::Foothold& foothold,
                             visualization_msgs::InteractiveMarker& marker);

  void publishKnots();

 private:
  void setupFootholdMarker(visualization_msgs::InteractiveMarker& marker,
                           const unsigned int stepNumber, const std::string& legName);

  void menuStepCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void menuTrajectoryCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void footholdCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  bool sendStepGoal();

  void print();

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Interactive marker server.
  interactive_markers::InteractiveMarkerServer server_;

  //! Menu handler.
  interactive_markers::MenuHandler menuHandler_;

  //! Step action client.
  std::unique_ptr<actionlib::SimpleActionClient<quadruped_msgs::StepAction>> stepActionClient_;

  //! TF listener.
  tf::TransformListener transformListener_;

  //! Foothold marker list.
  std::vector<FreeGaitMarker::Foothold> footholdList_;

  //! Topic name of the step action server.
  std::string actionServerTopic_;

  //! Timeout duration for the connection to the action server.
  ros::Duration waitForActionTimeout_;

  //! Foothold frame id.
  std::string footholdFrameId_;

  //! Foothold marker scale.
  double footholdScale_;

  //! Foothold marker radius.
  double footholdRadius_;

  //! Foothold marker color.
  std_msgs::ColorRGBA footholdColor_;

  //! Knot markers.
  std::vector<visualization_msgs::MarkerArray> knots_;

  //! Trajectory points markers.
  std::vector<visualization_msgs::MarkerArray> trajectories_;

  //! Knot ids.
  std::vector<int> knotIds_;

  //! Trajectory points ids.
  std::vector<int> trajectoryIds_;

  //! Container of curves.
  std::vector<robotUtils::PolynomialSplineVectorSpaceCurve<robotUtils::PolynomialSplineQuintic, 3>> splines_;

  //! Knot publisher.
  ros::Publisher knotsPublisher_;

  //! Trajectory publisher.
  ros::Publisher trajectoriesPublisher_;
};

} /* namespace free_gait_marker */
