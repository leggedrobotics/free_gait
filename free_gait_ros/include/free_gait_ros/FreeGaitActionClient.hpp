/*
 * FreeGaitActionClient.hpp
 *
 *  Created on: Oct 21, 2015
 *      Author: Georg Wiedebach, Péter Fankhauser
 */

#pragma once

#include <free_gait_msgs/ExecuteStepsAction.h>
#include <actionlib/client/simple_action_client.h>

namespace free_gait {

class FreeGaitActionClient {
public:
	enum ActionState {
		PENDING,
		ACTIVE,
		DONE
	};

	FreeGaitActionClient(ros::NodeHandle& nodeHandle, const std::string& name);
	FreeGaitActionClient(ros::NodeHandle& nodeHandle);
	virtual ~FreeGaitActionClient() {};

  void sendGoal(const free_gait_msgs::ExecuteStepsGoal& goal);
	void waitForResult(double timeout = 1e6); // TODO
	const free_gait_msgs::ExecuteStepsResult& getResult();
	ActionState getState();

protected:
	virtual void activeCallback() {};
	virtual void feedbackCallback(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback) {};
	virtual void doneCallback(const actionlib::SimpleClientGoalState& state, const free_gait_msgs::ExecuteStepsResultConstPtr& result){};

private:
	void activeCallback_();
	void feedbackCallback_(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback);
	void doneCallback_(const actionlib::SimpleClientGoalState& state, const free_gait_msgs::ExecuteStepsResultConstPtr& result);

	ros::NodeHandle& nodeHandle_;
	std::unique_ptr<actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>> client_;
	ActionState state_;
	free_gait_msgs::ExecuteStepsResult result_;
};

}
