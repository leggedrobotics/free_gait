#! /usr/bin/env python

import roslib
roslib.load_manifest('free_gait_action_loader')
from math import cos, sin
from free_gait_action_loader import *
from free_gait import *
import rospy
import tf
import actionlib
import free_gait_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg
import std_srvs.srv
import anymal_msgs.srv
import traceback
from actionlib_msgs.msg import *
import threading

global client

class ActionLoader:

    def __init__(self):
        self.request = None
        self._load_parameters()
        self.client = actionlib.SimpleActionClient(self.action_server_topic, free_gait_msgs.msg.ExecuteStepsAction)
        self.action_list = ActionList()
        self.action_list.update()
        self.action = None
        self.directory = None # Action's directory.

    def _load_parameters(self):
        self.action_server_topic = rospy.get_param('/free_gait/action_server')
        self.collection = rospy.get_param('~collection')

    def update_actions(self, request):
        success = self.action_list.update()
        response = std_srvs.srv.TriggerResponse()
        response.success = success
        return response

    def list_actions(self, request):
        ids = self.action_list.get_list_of_ids()
        response = anymal_msgs.srv.GetAvailableControllersResponse(ids)
        return response

    def send_action(self, request):
        self.reset()
        self.request = request
        response = anymal_msgs.srv.SwitchControllerResponse()
        action_entry = self.action_list.get(request.name)
        if action_entry.file is None:
            rospy.logerr('Action with name "' + request.name + '" does not exists.')
            response.status = response.STATUS_NOTFOUND
        else:
            try:
                self.directory = action_entry.directory

                if action_entry.type == ActionType.YAML:
                    self._load_yaml_action(action_entry.file)
                elif action_entry.type == ActionType.PYTHON:
                    self._load_python_action(action_entry.file)

                self.action.wait_for_result()

                if self.action.result is None:
                    response.status = response.STATUS_ERROR
                    rospy.logerr('An error occurred while reading the action.')
                    return response

                if self.action.result.status == free_gait_msgs.msg.ExecuteStepsResult.RESULT_FAILED:
                    response.status = response.STATUS_ERROR
                    rospy.logerr('An error occurred while executing the action.')
                    return response

                if self.action.keep_alive:
                    rospy.loginfo("Action continues in the background.")
                else:
                    rospy.loginfo('Action successfully executed.')
                response.status = response.STATUS_SWITCHED
                
            except:
                rospy.logerr('An exception occurred while reading the action.')
                response.status = response.STATUS_ERROR
                rospy.logerr(traceback.print_exc())

        return response

    def _load_yaml_action(self, file_path):
        # Load action from YAML file.
        rospy.loginfo('Loading free gait action from YAML file "' + file_path + '".')
        goal = load_action_from_file(file_path)
        rospy.logdebug(goal)
        if goal is None:
            rospy.logerr('Could not load action from YAML file.')
        self.action = SimpleAction(self.client, goal)

    def _load_python_action(self, file_path):
        # Load action from Python script.
        rospy.loginfo('Loading free gait action from Python script "' + file_path + '".')
        # action_locals = dict()
        # Kind of nasty, but currently only way to make external imports work.
        execfile(file_path, globals(), globals())
        self.action = action

    def check_and_start_action(self):
        if self.action is not None:
            if self.action.state == ActionState.INITIALIZED:
                self.action.start()

    def reset(self):
        if self.action:
            self.action.stop()
        del self.action
        self.action = None
        if self.client.gh:
            self.client.stop_tracking_goal()

    def preempt(self):
        try:
            if self.client.gh:
                if self.client.get_state() == GoalStatus.ACTIVE or self.client.get_state() == GoalStatus.PENDING:
                    self.client.cancel_all_goals()
                    rospy.logwarn('Canceling action.')
        except NameError:
            rospy.logerr(traceback.print_exc())


if __name__ == '__main__':
    try:
        rospy.init_node('free_gait_action_loader')
        action_loader = ActionLoader()
        rospy.on_shutdown(action_loader.preempt)

        rospy.Service('~update_actions', std_srvs.srv.Trigger, action_loader.update_actions)
        rospy.Service('~list_actions', anymal_msgs.srv.GetAvailableControllers, action_loader.list_actions)
        rospy.Service('~send_action', anymal_msgs.srv.SwitchController, action_loader.send_action)
        rospy.loginfo("Ready to load actions from service call.")

        updateRate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # This is required for having the actions run in the main thread
            # (instead through the thread by the service callback).
            action_loader.check_and_start_action()
            updateRate.sleep()

    except rospy.ROSInterruptException:
        pass
