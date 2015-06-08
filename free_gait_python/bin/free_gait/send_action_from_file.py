#! /usr/bin/env python

import roslib
from copy import deepcopy
from math import cos, sin
roslib.load_manifest('free_gait_python')
import rospy
import actionlib
import quadruped_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg
from free_gait import ActionConverter
import os

def step_client():
    client = actionlib.SimpleActionClient('/locomotion_controller/step', quadruped_msgs.msg.StepAction)
    client.wait_for_server()

    directory = os.path.dirname(__file__)
    file_path = os.path.abspath(os.path.join(directory, '../../actions/trot.yaml'))
    if not os.path.isfile(file_path):
        rospy.logerr('File with path "' + file_path + '" does not exists.')
        return
    
    goal = ActionConverter.load_from_file(file_path)
    
    client.send_goal(goal, feedback_cb=feedback_callback)
    print "Goal sent. Waiting for result."
    client.wait_for_result()
    return client.get_result()

def feedback_callback(feedback):
    print "Feedback:"
    print feedback

if __name__ == '__main__':
    try:
        rospy.init_node('send_action_from_file')
        result = step_client()
        print "Result:"
        print result
    except rospy.ROSInterruptException:
        print "Program interrupted before completion."
