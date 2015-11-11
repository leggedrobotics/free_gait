/*
 * StepGoalTools.cpp
 *
 *  Created on: Nov 11, 2015
 *      Author: Georg Wiedebach
 */

#include "free_gait_ros/StepGoalTools.hpp"

namespace free_gait {

void StepGoalTools::invertGoal(free_gait_msgs::StepGoal& goal) {
	std::reverse(goal.steps.begin(), goal.steps.end());

	for(std::vector<free_gait_msgs::Step>::iterator step_it = goal.steps.begin(); step_it != goal.steps.end(); ++step_it) {
		free_gait_msgs::Step& step = *step_it;

		for (free_gait_msgs::SwingData& swing_data : step.swing_data) {
			if (!swing_data.joint_trajectory.points.empty()) {
				ros::Duration total_time = swing_data.joint_trajectory.points.back().time_from_start;
				swing_data.joint_trajectory.points.pop_back();
				std::reverse(swing_data.joint_trajectory.points.begin(), swing_data.joint_trajectory.points.end());

				bool found_next = false;
				std::vector<free_gait_msgs::Step>::iterator next_step_it = step_it + 1;
				while (!found_next && std::distance(next_step_it, goal.steps.end())) {
					free_gait_msgs::Step& next_step = *next_step_it;
					for (free_gait_msgs::SwingData& next_swing_data : next_step.swing_data) {
						if (swing_data.name == next_swing_data.name) {
							swing_data.joint_trajectory.points.push_back(next_swing_data.joint_trajectory.points.back());
							found_next = true;
						}
					}
					++next_step_it;
				}

				for (trajectory_msgs::JointTrajectoryPoint& point : swing_data.joint_trajectory.points) {
					point.time_from_start = total_time - point.time_from_start;
				}
				if (!swing_data.joint_trajectory.points.empty()) {
					swing_data.joint_trajectory.points.back().time_from_start = total_time;
				}
			}

			if (!swing_data.foot_trajectory.points.empty()) {
				ROS_WARN("inverting foot trajectories is not implemented!");
				goal.steps.clear();
				return;
			}
		}

		for (free_gait_msgs::BaseShiftData& base_shift_data : step.base_shift_data) {
			if (!base_shift_data.trajectory.points.empty()) {
				ROS_WARN("inverting base shift trajectories is not implemented!");
				goal.steps.clear();
				return;
			}
		}
	}
}

} /* namespace free_gate */
