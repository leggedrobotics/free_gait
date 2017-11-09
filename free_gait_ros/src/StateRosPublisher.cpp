/*
 * StateRosPublisher.cpp
 *
 *  Created on: Dec 6, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_ros/StateRosPublisher.hpp>

// KDL
#include <kdl_parser/kdl_parser.hpp>

// Kindr
#include <kindr_ros/kindr_ros.hpp>

// STD
#include <string>
#include <vector>

#include <urdf/model.h>

namespace free_gait {

StateRosPublisher::StateRosPublisher(ros::NodeHandle& nodeHandle,
                                     AdapterBase& adapter)
    : nodeHandle_(nodeHandle),
      adapter_(adapter)
{
  tfPrefix_ = nodeHandle_.param("/free_gait/preview_tf_prefix", std::string(""));
  initializeRobotStatePublisher();
}

StateRosPublisher::~StateRosPublisher()
{
}

StateRosPublisher::StateRosPublisher(const StateRosPublisher& other) :
    nodeHandle_(other.nodeHandle_),
    tfPrefix_(other.tfPrefix_),
    adapter_(other.adapter_),
    tfBroadcaster_(other.tfBroadcaster_)
{
  if (other.robotStatePublisher_) {
    robotStatePublisher_.reset(
        new robot_state_publisher::RobotStatePublisher(*other.robotStatePublisher_));
  }
}

void StateRosPublisher::setTfPrefix(const std::string tfPrefix)
{
  tfPrefix_ = tfPrefix;
}

bool StateRosPublisher::initializeRobotStatePublisher()
{
  std::string robotDescriptionPath;
  if (nodeHandle_.hasParam("/free_gait/robot_description")) {
    nodeHandle_.getParam("/free_gait/robot_description", robotDescriptionPath);
  } else {
    ROS_ERROR("Did not find ROS parameter for robot description '/free_gait/robot_description'.");
    return false;
  }

  urdf::Model model;
  if (!model.initParam(robotDescriptionPath)) return false;

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)){
    ROS_ERROR("Failed to extract KDL tree from XML robot description");
    return false;
  }

  robotStatePublisher_.reset(new robot_state_publisher::RobotStatePublisher(tree));
  return true;
}

bool StateRosPublisher::publish(const State& state)
{
  const ros::Time time = ros::Time::now();

  // Publish joint states.
  std::vector<std::string> jointNames;
  state.getAllJointNames(jointNames);
  JointPositions jointPositions = state.getJointPositions();

  if (jointNames.size() != jointPositions.vector().size()) {
    ROS_ERROR("Joint name vector and joint position are not of equal size!");
    return false;
  }

  std::map<std::string, double> jointPositionsMap;
  for (size_t i = 0; i < jointNames.size(); ++i) {
    jointPositionsMap[jointNames[i]] = jointPositions(i);
  }

  robotStatePublisher_->publishTransforms(jointPositionsMap, time, tfPrefix_);
  robotStatePublisher_->publishFixedTransforms(tfPrefix_);

  // Publish base position.
  geometry_msgs::TransformStamped tfTransform;
  tfTransform.header.stamp = time;
  tfTransform.header.frame_id = adapter_.getWorldFrameId();
  tfTransform.child_frame_id = tf::resolve(tfPrefix_, adapter_.getBaseFrameId());
  kindr_ros::convertToRosGeometryMsg(state.getPositionWorldToBaseInWorldFrame(), tfTransform.transform.translation);
  kindr_ros::convertToRosGeometryMsg(state.getOrientationBaseToWorld(), tfTransform.transform.rotation);
  tfBroadcaster_.sendTransform(tfTransform);

  // Publish frame transforms.
  std::vector<std::string> frameTransforms;
  adapter_.getAvailableFrameTransforms(frameTransforms);
  for (const auto& frameId : frameTransforms) {
    geometry_msgs::TransformStamped tfTransform;
    tfTransform.header.stamp = time;
    tfTransform.header.frame_id = adapter_.getWorldFrameId();
    tfTransform.child_frame_id = tf::resolve(tfPrefix_, frameId);
    kindr_ros::convertToRosGeometryMsg(adapter_.getFrameTransform(frameId), tfTransform.transform);
    tfBroadcaster_.sendTransform(tfTransform);
  }

  return true;
}

//void StateRosPublisher::publishSupportRegion(const State& state)

} /* namespace free_gait */
