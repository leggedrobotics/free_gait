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
from free_gait import *
from os import listdir
from os.path import *
global client

def load_yaml_action(file_path):
    # Load action from YAML file.
    print "PENNISSSSSSSSSS"
    rospy.loginfo('Loading free gait action from "' + file_path + '".')
    print "PENNISSSSSSSSSS222222"
    goal = load_from_file(file_path, default_frame_id)
    print "goal:"
    print goal
    print "finis goal====="
    if goal == None:
        rospy.logerr('Could not load action.')
    return goal

def send_goal(goal):
    global client
    print goal
    
    action_server_topic = rospy.get_param('~action_server')
    default_frame_id = rospy.get_param('~default_frame_id')
    
    client = actionlib.SimpleActionClient(action_server_topic, quadruped_msgs.msg.StepAction)
    client.wait_for_server()
        
    print goal

    # Send action.
    client.send_goal(goal, feedback_cb=feedback_callback)
    rospy.loginfo('Goal sent. Waiting for result.')
    client.wait_for_result()
    return client.get_result()

def feedback_callback(feedback):
    pass
    #print "Feedback:"
    #print feedback

def send_action(request):
    response = locomotion_controller_msgs.srv.SwitchControllerResponse()
    file_path = get_path(request.name)
    if file_path == None:
        rospy.logerr('Action with name "' + file_path + '" does not exists.')
        response.status = response.STATUS_NOTFOUND
        return response
    
    else:
    #try:
        print "PENNIS1"
        goal = load_yaml_action(file_path)
        print "PENNIS2"
        if goal == None:
            rospy.logerror("Could not parse free gait action from YAML file.")
            response.status = response.STATUS_ERROR
            return response
        result = send_goal(goal)
        rospy.loginfo("Result:")
        rospy.loginfo(result)
        response.status = response.STATUS_SWITCHED
    #except:
     #   response.status = response.STATUS_ERROR
     #   return response
    
    return response

def get_path(file):
    directory = rospy.get_param('~directory')
    file_path = abspath(join(directory, file + '.yaml'))
    if not isfile(file_path):
        return None
    return file_path

def list_actions_service(request):
    path = rospy.get_param('~directory')
    actions = [ splitext(f)[0] for f in listdir(path) if isfile(join(path,f)) ]
    actions.sort()
    response = locomotion_controller_msgs.srv.GetAvailableControllersResponse(actions)
    return response

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
        
        # Decide what to do.
        if rospy.has_param('~file'):
            file = rospy.get_param('~file')
            rospy.delete_param('~file')
            request = locomotion_controller_msgs.srv.SwitchControllerRequest(file)
            send_action(request)
            rospy.signal_shutdown("Action sent, shutting down.")
        else:
            rospy.Service('~send_action', locomotion_controller_msgs.srv.SwitchController, send_action)
            rospy.Service('~list_actions', locomotion_controller_msgs.srv.GetAvailableControllers, list_actions_service)
            rospy.loginfo("Ready to load actions from service call.")
            rospy.spin()
            
        
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted before completion.")
