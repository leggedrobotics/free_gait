/*
 * free_gait_marker_node.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "free_gait_marker/FreeGaitMarker.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "free_gait_marker");

  ros::NodeHandle nodeHandle("~");

  free_gait_marker::FreeGaitMarker freeGaitMarker(nodeHandle);

  ros::Rate loop_rate(30);
  int count = 0;
  while (ros::ok()) {
    ros::spinOnce();
    freeGaitMarker.publishKnots();
    loop_rate.sleep();
  }

  return 0;
}
