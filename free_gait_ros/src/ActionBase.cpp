/*
 * ActionBase.cpp
 *
 *  Created on: Oct 21, 2015
 *      Author: Georg Wiedebach
 */

#include "free_gait_ros/ActionBase.hpp"

namespace free_gait {

ActionBase::ActionBase(actionlib::SimpleActionClient<free_gait_msgs::StepAction>* client) {
	client_ = client;
	state_ = ActionState::DONE;
}

ActionBase::ActionBase(actionlib::SimpleActionClient<free_gait_msgs::StepAction>* client, free_gait_msgs::StepGoal goal)
	: ActionBase(client) {
	goal_ = goal;
}

void ActionBase::start() {
	sendGoal();
}

void ActionBase::sendGoal() {
	state_ = ActionState::PENDING;
	client_->cancelAllGoals();
	client_->waitForServer();
	client_->sendGoal(goal_, boost::bind(&ActionBase::doneCallback_, this, _1, _2),
			boost::bind(&ActionBase::activeCallback_, this),
			boost::bind(&ActionBase::feedbackCallback_, this, _1));
}

void ActionBase::waitForResult(double timeout) {
	client_->waitForResult(ros::Duration(timeout));
}

free_gait_msgs::StepResult ActionBase::getResult() {
	return result_;
}

ActionBase::ActionState ActionBase::getState() {
	return state_;
}

void ActionBase::doneCallback_(const actionlib::SimpleClientGoalState& state, const free_gait_msgs::StepResultConstPtr& result) {
	state_ = ActionState::DONE;
	result_ = *result;
	doneCallback(state, result);
}

void ActionBase::activeCallback_() {
	state_ = ActionState::ACTIVE;
	activeCallback();
}

void ActionBase::feedbackCallback_(const free_gait_msgs::StepFeedbackConstPtr& feedback) {
	feedbackCallback(feedback);
}

}
