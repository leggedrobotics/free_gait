/*
 * StepGoalTools.hpp
 *
 *  Created on: Nov 11, 2015
 *      Author: Georg Wiedebach
 */

#ifndef FREE_GAIT_FREE_GAIT_ROS_SRC_STEPGOALTOOLS_HPP_
#define FREE_GAIT_FREE_GAIT_ROS_SRC_STEPGOALTOOLS_HPP_

#include "free_gait_msgs/StepGoal.h"
#include "ros/ros.h"

namespace free_gait {

class StepGoalTools {
public:
	StepGoalTools() {};
	virtual ~StepGoalTools() {};

	void invertGoal(free_gait_msgs::StepGoal& goal);
};

} /* namespace free_gate */

#endif /* FREE_GAIT_FREE_GAIT_ROS_SRC_STEPGOALTOOLS_HPP_ */
