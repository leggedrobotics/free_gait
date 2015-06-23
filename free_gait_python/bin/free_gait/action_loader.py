#! /usr/bin/env python

import roslib
from math import cos, sin
roslib.load_manifest('free_gait_python')
import rospy
import tf
import actionlib
import quadruped_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg
import locomotion_controller_msgs.srv
import traceback
from free_gait import *
from os import listdir
from os.path import *

global client

class ActionLoader:
    
    def __init__(self):
        self.default_frame_id = 'map'
        self.action_server_topic = '/locomotion_controller/step'
        self._load_parameters()
        
    def _load_parameters(self):
        self.default_frame_id = rospy.get_param('~default_frame_id')
        self.action_server_topic = rospy.get_param('~action_server')
        self.directory = rospy.get_param('~directory')
    
    def _get_path(self, file):
        directory = rospy.get_param('~directory')
        file_path = abspath(join(directory, file))
        if not isfile(file_path):
            return None
        return file_path
        
    def send_action(self, request):
        response = locomotion_controller_msgs.srv.SwitchControllerResponse()
        file_path = self._get_path(request.name)
        file_type = splitext(request.name)[-1]
        if file_path == None:
            rospy.logerr('Action with name "' + request.name + '" does not exists.')
            response.status = response.STATUS_NOTFOUND
        else:
            try:
                if file_type == '.yaml':
                    goal = self.load_yaml_action(file_path)
                elif file_type == '.py':
                    goal = self.load_python_action(file_path)
                    
                if goal == None:
                    response.status = response.STATUS_ERROR
                    return response
                result = self.send_goal(goal)
                rospy.loginfo('Result:')
                rospy.loginfo(result)
                response.status = response.STATUS_SWITCHED
            except:
                rospy.logerr('A parser error occurred.')
                response.status = response.STATUS_ERROR
                traceback.print_exc()
            
        return response

    def load_yaml_action(self, file_path):
        # Load action from YAML file.
        rospy.loginfo('Loading free gait action from YAML file "' + file_path + '".')
        goal = load_from_file(file_path, self.default_frame_id)
        if goal == None:
            rospy.logerr('Could not load action from YAML file.')
        return goal
    
    def load_python_action(self, file_path):
        # Load action from Python script.
        rospy.loginfo('Loading free gait action from Python script "' + file_path + '".')
        
        action_locals = dict()
        execfile(file_path, dict(), action_locals)
        goal = action_locals['goal']
        if goal == None:
            rospy.logerr('Could not load action from Python script.')
        return goal
    
    def send_goal(self, goal):
        client = actionlib.SimpleActionClient(self.action_server_topic, quadruped_msgs.msg.StepAction)
        client.wait_for_server()
#         print goal
    
        # Send action.
        client.send_goal(goal, feedback_cb=self.feedback_callback)
        rospy.loginfo('Goal sent. Waiting for result.')
        client.wait_for_result()
        return client.get_result()
    
    def list_actions_service(self, request):
        actions = [ f for f in listdir(self.directory) if isfile(join(self.directory, f)) ]
        actions.sort()
        response = locomotion_controller_msgs.srv.GetAvailableControllersResponse(actions)
        return response
    
    def feedback_callback(self, feedback):
        pass
         #print "Feedback:"
         #print feedback
    
def preempt():
    global client
    try:
        if client.get_state() == GoalStatus.ACTIVE or client.get_state() == GoalStatus.PENDING:
            client.cancel_goal()
            rospy.logwarn('Canceling action.')
    except NameError:
        pass


if __name__ == '__main__':
    try:
        rospy.init_node('free_gait_action_loader')
        rospy.on_shutdown(preempt)
        action_loader = ActionLoader()
        
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
            rospy.Service('~list_actions', locomotion_controller_msgs.srv.GetAvailableControllers, action_loader.list_actions_service)
            rospy.loginfo("Ready to load actions from service call.")
            rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted before completion.")
