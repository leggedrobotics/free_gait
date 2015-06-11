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
global print_feedback

def step_client():
    global print_feedback
    file_path = rospy.get_param('~file_path');
    action_server_topic = rospy.get_param('~action_server');
    map_frame_id = rospy.get_param('~map_frame_id');
    foot_frame_id = rospy.get_param('~foot_frame_id');
    print_feedback = rospy.get_param('~print_feedback', 'false');
    
    client = actionlib.SimpleActionClient(action_server_topic, quadruped_msgs.msg.StepAction)
    client.wait_for_server()

    # Get current pose of robot to adapt action
    # to the current coordinates if necessary.
    listener = tf.TransformListener()
    listener.waitForTransform(map_frame_id, foot_frame_id, rospy.Time(0), rospy.Duration(10.0))
    try:
        (translation, rotation) = listener.lookupTransform(map_frame_id, foot_frame_id, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr('Could not look up TF transformation from "' +
                     map_frame_id + '" to "' + foot_frame_id + '".')
        return
    
    # Load action from YAML file.
    #directory = os.path.dirname(__file__)
    #file_path = os.path.abspath(os.path.join(directory, '../../actions/example.yaml'))
    rospy.loginfo('Loading free gait action from "' + file_path + '".')
    goal = load_from_file(file_path, translation, rotation)
#     print goal

    # Send action.
    client.send_goal(goal, feedback_cb=feedback_callback)
    rospy.loginfo('Goal sent. Waiting for result.')
    client.wait_for_result()
    return client.get_result()

def feedback_callback(feedback):
    global print_feedback
    if print_feedback:
        print "Feedback:"
        print feedback

if __name__ == '__main__':
    try:
        rospy.init_node('free_gait_action_from_file')
        result = step_client()
        print "Result:"
        print result
    except rospy.ROSInterruptException:
        print "Program interrupted before completion."
