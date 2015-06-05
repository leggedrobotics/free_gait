#! /usr/bin/env python

import roslib
from copy import deepcopy
roslib.load_manifest('free_gait_scripts')
import rospy
import actionlib
import quadruped_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

def step_client():
    client = actionlib.SimpleActionClient('/locomotion_controller/step', quadruped_msgs.msg.StepAction)
    client.wait_for_server()
    
    goal = quadruped_msgs.msg.StepGoal()
    
    step1 = quadruped_msgs.msg.StepData()
    step1.step_number = 1
    step1.leg_names.append("rightHind")
    step1.leg_names.append("leftFore")
    step1.swing_durations.append(rospy.Duration(0.3))
    step1.swing_durations.append(step1.swing_durations[0])
    step1.positions.append(geometry_msgs.msg.Point(-0.279, -0.218, 0.0249))
    step1.positions.append(geometry_msgs.msg.Point(0.279, 0.218, 0.0249))
    step1.normals.append(geometry_msgs.msg.Vector3(0.0, 0.0, 1.0))
    step1.normals.append(step1.normals[0])
    step1.swing_heights.append(0.12)
    step1.swing_heights.append(step1.swing_heights[0])
    step1.state_names.append("pre_step")
    step1.state_names.append("post_step")
    step1.shift_durations.append(rospy.Duration(0.0))
    step1.shift_durations.append(step1.shift_durations[0])
    goal.steps.append(step1)
    
    step2 = deepcopy(step1)
    step2.step_number = 2
    step2.leg_names[0] = "leftHind"
    step2.leg_names[1] = "rightFore"
    step2.positions[0] = geometry_msgs.msg.Point(-0.279, 0.218, 0.0249)
    step2.positions[1] = geometry_msgs.msg.Point(0.279, -0.218, 0.0249)
    goal.steps.append(step2)
     
    for i in range(3, 20, 2):
        step = deepcopy(step1)
        step.step_number = i
        goal.steps.append(step)
        step = deepcopy(step2)
        step.step_number = i+1
        goal.steps.append(step)
    
    print "Goal:"
    print goal
    
    client.send_goal(goal, feedback_cb=feedback_callback)
    print "Goal sent. Waiting for result."
    client.wait_for_result()
    return client.get_result()

def feedback_callback(feedback):
    print "Feedback:"
    print feedback

if __name__ == '__main__':
    try:
        rospy.init_node('step_test')
        result = step_client()
        print "Result:"
        print result
    except rospy.ROSInterruptException:
        print "Program interrupted before completion."
