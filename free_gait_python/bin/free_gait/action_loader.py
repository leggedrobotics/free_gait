#! /usr/bin/env python

import roslib
roslib.load_manifest('free_gait_python')
from math import cos, sin
import rospy
import tf
import actionlib
import free_gait_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg
import locomotion_controller_msgs.srv
import traceback
from actionlib_msgs.msg import *
from free_gait import *
from os import listdir
from os.path import *

global client

class ActionLoader:
    
    def __init__(self):
        self.default_frame_id = 'map'
        self.action_server_topic = '/locomotion_controller/step'
        self.request = None
        self._load_parameters()
        self.client = actionlib.SimpleActionClient(self.action_server_topic, free_gait_msgs.msg.StepAction)
        self.action = None
        
    def _load_parameters(self):
        self.default_frame_id = rospy.get_param('~default_frame_id')
        self.action_server_topic = rospy.get_param('~action_server')
        self.directory = rospy.get_param('~directory')
        
    def list_actions(self, request):
        actions = [ f for f in listdir(self.directory) if isfile(join(self.directory, f)) ]
        actions.sort()
        response = locomotion_controller_msgs.srv.GetAvailableControllersResponse(actions)
        return response
        
    def send_action(self, request):
        self.request = request
        response = locomotion_controller_msgs.srv.SwitchControllerResponse()
        file_path = self._get_path(request.name)
        file_type = splitext(request.name)[-1]
        if file_path == None:
            rospy.logerr('Action with name "' + request.name + '" does not exists.')
            response.status = response.STATUS_NOTFOUND
        else:
            try:
                if file_type == '.yaml':
                    goal = self._load_yaml_action(file_path)
                    self.feedback_trigger = None
                elif file_type == '.py':
                    self._load_python_action(file_path)
                    
                if self.action.goal == None:
                    response.status = response.STATUS_ERROR
                    return response
                
                self.action.send_goal()
                rospy.loginfo('Goal sent. Waiting for result.' + self.client.get_goal_status_text())
                self.action.wait_for_result()
                result = self.action.get_result()
                                
                if result == None:
                    response.status = response.STATUS_ERROR
                    return response
                rospy.loginfo('Result:')
                rospy.loginfo(result)
                response.status = response.STATUS_SWITCHED
            except:
                rospy.logerr('An error occurred while reading the action.')
                response.status = response.STATUS_ERROR
                rospy.logerr(traceback.print_exc())
                
        return response
    
    def _get_path(self, file):
        directory = rospy.get_param('~directory')
        file_path = abspath(join(directory, file))
        if not isfile(file_path):
            return None
        return file_path

    def _load_yaml_action(self, file_path):
        # Load action from YAML file.
        rospy.loginfo('Loading free gait action from YAML file "' + file_path + '".')
        self.action = SimpleAction(self.client, load_from_file(file_path, self.default_frame_id))
        if self.action.goal == None:
            rospy.logerr('Could not load action from YAML file.')
    
    def _load_python_action(self, file_path):
        # Load action from Python script.
        rospy.loginfo('Loading free gait action from Python script "' + file_path + '".')
        action_locals = dict()
#         action_globals = {'feedback' : feedback, 'rospy' : rospy}
        execfile(file_path, globals(), action_locals)
        self.action = action_locals['action']
        if self.action.goal == None:
            rospy.logerr('Could not load action from Python script.')
                
    def preempt(self):
        try:
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
        
        # Decide what to do.
        load_file = False
        if rospy.has_param('~file'):
            file = rospy.get_param('~file')
            if file != "":
                load_file = True
            rospy.delete_param('~file')
            
        if load_file:
            request = locomotion_controller_msgs.srv.SwitchControllerRequest(file)
            action_loader.send_action(request)
            rospy.signal_shutdown("Action sent, shutting down.")
        else:
            rospy.Service('~send_action', locomotion_controller_msgs.srv.SwitchController, action_loader.send_action)
            rospy.Service('~list_actions', locomotion_controller_msgs.srv.GetAvailableControllers, action_loader.list_actions)
            rospy.loginfo("Ready to load actions from service call.")
            rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted before completion.")
