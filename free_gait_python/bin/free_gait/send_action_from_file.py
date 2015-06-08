#! /usr/bin/env python

import roslib
from copy import deepcopy
from math import cos, sin
roslib.load_manifest('free_gait_python')
import rospy
import tf
import actionlib
import quadruped_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg
from free_gait import *
import os

def step_client():
    client = actionlib.SimpleActionClient('/locomotion_controller/step', quadruped_msgs.msg.StepAction)
    client.wait_for_server()

    # Get current pose of robot to adapt action
    # to the current coordinates if necessary.
    listener = tf.TransformListener()
    listener.waitForTransform('map', 'footprint', rospy.Time(0), rospy.Duration(10.0))
    try:
        (translation, rotation) = listener.lookupTransform('map', 'footprint', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr('Could not look up TF transformation from "map" to "footprint".')
        return
    
    # Load action from YAML file.
    directory = os.path.dirname(__file__)
    file_path = os.path.abspath(os.path.join(directory, '../../actions/trot.yaml'))
    rospy.loginfo('Loading free gait action from "' + file_path + '".')
    goal = load_from_file(file_path, translation, rotation)

    # Send action.
    client.send_goal(goal, feedback_cb=feedback_callback)
    rospy.loginfo('Goal sent. Waiting for result.')
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
