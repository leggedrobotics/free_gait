#! /usr/bin/env python

import roslib
roslib.load_manifest('free_gait_action_loader')
from free_gait_action_loader import *
from free_gait import *
import rospy
import actionlib
import free_gait_msgs.msg
import free_gait_msgs.srv
import std_srvs.srv
import traceback
from actionlib_msgs.msg import *
import threading

class ActionLoader:

    def __init__(self):
        self.name = rospy.get_name()[1:]
        self.action_list = ActionList(self.name)
        self.action_list.update()
        self.collection_list = CollectionList(self.name)
        self.collection_list.update()
        self.action = None
        self.directory = None # Action's directory.

        step_action_server_topic = rospy.get_param('/free_gait/action_server')
        self.execute_steps_client = actionlib.SimpleActionClient(step_action_server_topic, free_gait_msgs.msg.ExecuteStepsAction)
        self.execute_action_server = actionlib.SimpleActionServer("~execute_action", free_gait_msgs.msg.ExecuteActionAction, \
                                                          execute_cb=self._execute_action_callback, auto_start = False)
        self.execute_action_server.start()

        rospy.Service('~update', std_srvs.srv.Trigger, self.update)
        rospy.Service('~list_actions', free_gait_msgs.srv.GetActions, self.list_actions)
        rospy.Service('~list_collections', free_gait_msgs.srv.GetCollections, self.list_collections)
        rospy.Service('~send_action', free_gait_msgs.srv.SendAction, self._send_action_callback)
        rospy.on_shutdown(self.preempt)

    def update(self, request):
        success = self.action_list.update() and self.collection_list.update()
        response = std_srvs.srv.TriggerResponse()
        response.success = success
        return response

    def list_actions(self, request):
        response = free_gait_msgs.srv.GetActionsResponse()
        action_ids = []
        if request.collection_id:
            collection = self.collection_list.get(request.collection_id)
            if collection is None:
                return response
            action_ids = collection.action_ids
        response.actions = self.action_list.to_ros_message(action_ids)
        return response

    def list_collections(self, request):
        response = free_gait_msgs.srv.GetCollectionsResponse()
        response.collections = self.collection_list.to_ros_message()
        return response

    def _execute_action_callback(self, goal):
        result = self.send_action(goal.action_id)
        if result.status != result.RESULT_NOT_FOUND:
            self.action.wait_for_state([ActionState.ERROR, ActionState.DONE])
        if self.action.state == ActionState.DONE:
            result.status = result.RESULT_DONE
        self.execute_action_server.set_succeeded(result)

    def _send_action_callback(self, request):
        response = free_gait_msgs.srv.SendActionResponse()
        response.result = self.send_action(request.goal.action_id)
        return response

    def send_action(self, action_id):
        self.reset()
        action_entry = self.action_list.get(action_id)
        result = free_gait_msgs.msg.ExecuteActionResult()

        if action_entry is None:
            rospy.logerr('Action with id "' + action_id + '" does not exists.')
            result.status = result.RESULT_NOT_FOUND
            return result

        if action_entry.file is None:
            rospy.logerr('File for action with id "' + action_id + '" does not exists.')
            result.status = result.RESULT_NOT_FOUND
            return result

        try:
            self.directory = action_entry.directory
            if action_entry.type == ActionType.YAML:
                self._load_yaml_action(action_entry.file)
            elif action_entry.type == ActionType.PYTHON:
                self._load_python_action(action_entry.file)

            if self.action is None:
                result.status = result.RESULT_UNKNOWN
                rospy.logerr('An unkown state has been reached while reading the action.')
                return result

            if self.action.state == ActionState.ERROR or self.action.state == ActionState.UNINITIALIZED:
                result.status = result.RESULT_FAILED
                rospy.logerr('An error occurred while loading the action.')
                return result

            self.action.register_callback(self._action_feedback_callback, self._action_done_callback)
            self.action.wait_for_state([ActionState.ERROR, ActionState.ACTIVE, ActionState.IDLE, ActionState.DONE])

            if self.action.state == ActionState.ERROR:
                result.status = result.RESULT_FAILED
                rospy.logerr('An error occurred while initializing the action.')
            else:
                result.status = result.RESULT_STARTED
                rospy.loginfo('Action was successfully started.')

        except:
            rospy.logerr('An exception occurred while loading the action.')
            result.status = result.RESULT_FAILED
            rospy.logerr(traceback.print_exc())

        return result

    def _load_yaml_action(self, file_path):
        # Load action from YAML file.
        rospy.loginfo('Loading free gait action from YAML file "' + file_path + '".')
        goal = load_action_from_file(file_path)
        rospy.logdebug(goal)
        self.action = SimpleAction(self.execute_steps_client, goal)
        if goal is None:
            rospy.logerr('Could not load action from YAML file.')
            self.action.set_state(ActionState.ERROR)

    def _load_python_action(self, file_path):
        # Load action from Python script.
        rospy.loginfo('Loading free gait action from Python script "' + file_path + '".')
        # action_locals = dict()
        # Kind of nasty, but currently only way to make external imports work.
        execfile(file_path, globals(), globals())
        self.action = action

    def _action_feedback_callback(self):
        rospy.loginfo('Action switched to state: ' + ActionState.to_text(self.action.state) + '.')
        if self.execute_action_server.is_active():
            feedback = free_gait_msgs.msg.ExecuteActionFeedback()
            feedback.status = self.action.state
            self.execute_action_server.publish_feedback(feedback)

    def _action_done_callback(self):
        rospy.loginfo('Action switched to state: ' + ActionState.to_text(self.action.state) + '.')

    def _check_and_start_action(self):
        if self.action is not None:
            if self.action.state == ActionState.INITIALIZED:
                self.action.start()

    def reset(self):
        if self.action:
            self.action.stop()
        del self.action
        self.action = None
        if self.execute_steps_client.gh:
            self.execute_steps_client.stop_tracking_goal()

    def preempt(self):
        try:
            if self.execute_steps_client.gh:
                if self.execute_steps_client.get_state() == GoalStatus.ACTIVE or self.execute_steps_client.get_state() == GoalStatus.PENDING:
                    self.execute_steps_client.cancel_all_goals()
                    rospy.logwarn('Canceling action.')
        except NameError:
            rospy.logerr(traceback.print_exc())


if __name__ == '__main__':
    try:
        rospy.init_node('free_gait_action_loader')
        action_loader = ActionLoader()
        rospy.loginfo('Ready to load actions from service call.')

        updateRate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # This is required for having the actions run in the main thread
            # (instead through the thread by the service callback).
            action_loader._check_and_start_action()
            updateRate.sleep()

    except rospy.ROSInterruptException:
        pass
