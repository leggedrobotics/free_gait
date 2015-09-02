/*
 * MarkerManager.hpp
 *
 *  Created on: Feb 28, 2015
 *      Author: Péter Fankhauser, Dario Bellicoso
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <free_gait_msgs/StepAction.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <std_msgs/ColorRGBA.h>

// Kindr ROS plugins
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

// STD
#include <string>
#include <vector>

// Curves
#include "curves/PolynomialSplineVectorSpaceCurve.hpp"

// Markers
#include "free_gait_marker/markers/MarkerKnot.hpp"
#include "free_gait_marker/markers/MarkerFoot.hpp"

namespace free_gait_marker {

class MarkerManager
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  MarkerManager(ros::NodeHandle& nodeHandle);

  virtual ~MarkerManager();

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

  void loadManagerParameters();

  //Note: This change will not take effect until you call applyChanges()
  void addFootholdMarker(const unsigned int stepNumber,
                         const std::string& legName);

  void addKnotMarker(const unsigned int markerNumber,
                     const std::string& markerName,
                     const geometry_msgs::Pose& pose);

  //Note: This change will not take effect until you call applyChanges()
  bool activateFoothold(Foothold& foothold);

  //Note: This change will not take effect until you call applyChanges()
  bool deactivateFoothold(Foothold& foothold);

  //Note: This change will not take effect until you call applyChanges()
  bool deactivateAllFootholds();

  void applyChanges();

  MarkerManager::Foothold& getFootholdByMarkerName(const std::string& markerName);

  bool getMarkerFromFoothold(const MarkerManager::Foothold& foothold,
                             markers::MarkerFoot& marker);

  void publishKnots();
  void updateKnots();

 private:
  void footholdCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void knotCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  bool sendStepGoal();

  void print();

  void setupMenus();

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Interactive marker server.
  interactive_markers::InteractiveMarkerServer server_;

  //! Menu handler.
  interactive_markers::MenuHandler footMenuHandler_;
  interactive_markers::MenuHandler knotMenuHandler_;

  //! Step action client.
  std::unique_ptr<actionlib::SimpleActionClient<free_gait_msgs::StepAction>> stepActionClient_;

  //! TF listener.
  tf::TransformListener transformListener_;

  //! Foothold marker list.
  std::vector<MarkerManager::Foothold> footholdList_;

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

  //! Trajectory points markers.
//  std::vector<visualization_msgs::MarkerArray> trajectories_;
  std::vector<trajectory_msgs::MultiDOFJointTrajectory> trajectories_;

  //! Knot ids.
  std::vector<int> knotIds_;

  //! Trajectory points ids.
  std::vector<int> trajectoryIds_;

  std::vector<bool> showTrajectory_;

  //! Container of curves.
  std::vector<curves::PolynomialSplineQuinticVector3Curve> splines_;

  //! Knot publisher.
  ros::Publisher knotsPublisher_;

  //! Trajectory publisher.
  ros::Publisher trajectoriesPublisher_;

  void clearTrajectory(int legId);
  void updateTrajectory(int legId, const std::string& markerName);


  /******************
   * Menu callbacks *
   ******************/
  //:: foothold marker callbacks
  void activateCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  // step
  void sendStepCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void resetCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  // trajectory
  void setKnotCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void sendTrajectoryCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void clearTrajectoryCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void showTrajectoryCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void deleteKnotsCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  //:: knot marker callbacks
  void knotMenuUpdateCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void knotMenuResetCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void knotMenuDeleteCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  /******************/

  void getIdAndNameFromMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
                              int& legId,
                              std::string& name);

};

} /* namespace free_gait_marker */
