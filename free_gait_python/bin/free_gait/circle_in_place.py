#! /usr/bin/env python

import roslib
from copy import deepcopy
from math import cos, sin
roslib.load_manifest('free_gait_scripts')
import rospy
import actionlib
import quadruped_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg

def step_client():
    client = actionlib.SimpleActionClient('/locomotion_controller/step', quadruped_msgs.msg.StepAction)
    client.wait_for_server()
    
    goal = quadruped_msgs.msg.StepGoal()
    angle = 0.35
    
    for cycle in range(0, 30):
        step = quadruped_msgs.msg.Step()
        step.step_number = 4 * cycle + 1
        swingData = quadruped_msgs.msg.SwingData()
        swingData.name = "rightFore";
        swingData.profile.target.header.frame_id = "map"
        swingData.profile.target.point = get_foot_position_for_angle(swingData.name, cycle * angle)
        step.swing_data.append(swingData)
        goal.steps.append(step)
        del step, swingData
        
        step = quadruped_msgs.msg.Step()
        step.step_number = 4 * cycle + 2
        swingData = quadruped_msgs.msg.SwingData()
        swingData.name = "leftFore";
        swingData.profile.target.header.frame_id = "map"
        swingData.profile.target.point = get_foot_position_for_angle(swingData.name, cycle * angle)
        step.swing_data.append(swingData)
        goal.steps.append(step)
        del step, swingData
        
        step = quadruped_msgs.msg.Step()
        step.step_number = 4 * cycle + 3
        swingData = quadruped_msgs.msg.SwingData()
        swingData.name = "leftHind";
        swingData.profile.target.header.frame_id = "map"
        swingData.profile.target.point = get_foot_position_for_angle(swingData.name, cycle * angle)
        step.swing_data.append(swingData)
        goal.steps.append(step)
        del step, swingData
        
        step = quadruped_msgs.msg.Step()
        step.step_number = 4 * cycle + 4
        swingData = quadruped_msgs.msg.SwingData()
        swingData.name = "rightHind";
        swingData.profile.target.header.frame_id = "map"
        swingData.profile.target.point = get_foot_position_for_angle(swingData.name, cycle * angle)
        step.swing_data.append(swingData)
        goal.steps.append(step)
        del step, swingData

    print goal
    
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
        rospy.init_node('circle_in_place')
        result = step_client()
        print "Result:"
        print result
    except rospy.ROSInterruptException:
        print "Program interrupted before completion."
