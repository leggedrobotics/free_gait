/*
 * ActionBase.hpp
 *
 *  Created on: Oct 21, 2015
 *      Author: Georg Wiedebach
 */

#ifndef WHOLE_BODY_CLIMBING_SRC_ACTIONBASE_HPP_
#define WHOLE_BODY_CLIMBING_SRC_ACTIONBASE_HPP_

#include <free_gait_msgs/ExecuteStepsAction.h>
#include <actionlib/client/simple_action_client.h>

namespace free_gait {

class ActionBase {
public:
	enum ActionState {
		PENDING,
		ACTIVE,
		DONE
	};

	ActionBase(actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>* client);
	ActionBase(actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>* client, free_gait_msgs::ExecuteStepsGoal goal);
	virtual ~ActionBase() {};

	void start();
	void waitForResult(double timeout);
	free_gait_msgs::ExecuteStepsResult getResult();
	ActionState getState();

protected:
	virtual void activeCallback() {};
	virtual void feedbackCallback(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback) {};
	virtual void doneCallback(const actionlib::SimpleClientGoalState& state, const free_gait_msgs::ExecuteStepsResultConstPtr& result){};

	free_gait_msgs::ExecuteStepsGoal goal_;

private:
	void sendGoal();
	void activeCallback_();
	void feedbackCallback_(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback);
	void doneCallback_(const actionlib::SimpleClientGoalState& state, const free_gait_msgs::ExecuteStepsResultConstPtr& result);

	actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>* client_;
	ActionState state_;
	free_gait_msgs::ExecuteStepsResult result_;
};

}

#endif /* WHOLE_BODY_CLIMBING_SRC_ACTIONBASE_HPP_ */
