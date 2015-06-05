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
    #client.wait_for_server()

    directory = os.path.dirname(__file__)
    file_path = os.path.abspath(os.path.join(directory, '../../actions/example.yaml'))
    goal = ActionConverter.load_from_file(file_path)
    
    client.send_goal(goal, feedback_cb=feedback_callback)
    print "Goal sent. Waiting for result."
    client.wait_for_result()
    return client.get_result()

def get_foot_position_for_angle(footName, angle):
    # Rotates the step position around the current base center by an angle.
    # TODO Get current base position.
    length = 0.2792
    width = 0.2189
    height = 0.025
    position = geometry_msgs.msg.Point();
    
    if footName == "rightFore":
        width = -width
    elif footName == "leftHind":
        length = -length
    elif footName == "rightHind":
        width = -width
        length = -length
        
    position.x = length * cos(angle) - width * sin(angle)
    position.y = length * sin(angle) + width * cos(angle)
    position.z = height
    print position
    return position;
    

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
