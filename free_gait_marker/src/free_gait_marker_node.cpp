/*
 * free_gait_marker_node.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: Péter Fankhauser, Dario Bellicoso
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>

#include "free_gait_marker/marker_manager/MarkerManager.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "free_gait_marker");

  ros::NodeHandle nodeHandle("~");

  free_gait_marker::MarkerManager markerManager(nodeHandle);

  ros::Rate loop_rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    markerManager.publishKnots();
    loop_rate.sleep();
  }

  return 0;
}
