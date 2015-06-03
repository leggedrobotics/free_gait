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
    
    stepNumber = 1;
    step1 = quadruped_msgs.msg.Step()
    step1.step_number = stepNumber
    swingData = quadruped_msgs.msg.SwingData()
    swingData.name = "leftHind"
    swingData.profile.target.header.frame_id = "map"
    swingData.profile.target.point = geometry_msgs.msg.Point(4.37, 6.06, 0.0249137096107);
    swingData.profile.height = 0.1
    swingData.profile.duration = rospy.Duration(1.5)
    step1.swing_data.append(swingData)
    goal.steps.append(step1)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "rightHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(4.37, 6.43, 0.0249135605991)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "leftFore"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.88, 6.06, 0.0249135605991)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "rightFore"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.88, 6.43, 0.0249135605991)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "leftHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(4.28, 6.06, 0.0249135605991)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "rightHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(4.28, 6.43, 0.0249135605991)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "leftFore"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.79, 6.06, 0.158406764269)
    step.swing_data[0].profile.height = 0.35
    step.swing_data[0].profile.duration = rospy.Duration(3.0)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "rightFore"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.79, 6.43, 0.153285458684)
    step.swing_data[0].profile.height = 0.35
    step.swing_data[0].profile.duration = rospy.Duration(3.0)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "leftFore"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.72, 6.06, 0.158406764269)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "rightFore"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.72, 6.43, 0.158406764269)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "rightHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(4.22, 6.43, 0.0249135605991)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "leftHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(4.22, 6.06, 0.0249135605991)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "leftFore"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.66, 6.06, 0.153285458684)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "rightFore"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.66, 6.43, 0.153285458684)
    goal.steps.append(step)
     
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "leftHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(4.15, 6.06, 0.0249135605991)
    goal.steps.append(step)
     
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "rightHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(4.15, 6.43, 0.0249135605991)
    goal.steps.append(step)
     
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "leftFore"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.56, 6.06, 0.111056029797)
    goal.steps.append(step)
     
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "rightFore"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.56, 6.43, 0.111056029797)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "leftHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.97, 6.06, 0.0249135605991)
    goal.steps.append(step)
     
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "rightHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.97, 6.43, 0.0249135605991)
    goal.steps.append(step)
 
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "leftFore"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.4, 6.06, 0.111056029797)
    goal.steps.append(step)

    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "rightFore"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.45, 6.43, 0.111056029797)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "leftHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.93, 6.06, 0.025)
    goal.steps.append(step)
     
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "rightHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.93, 6.43, 0.025)
    goal.steps.append(step)
        
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "leftHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.76, 6.06, 0.17)
    step.swing_data[0].profile.height = 0.35
    step.swing_data[0].profile.duration = rospy.Duration(3.0)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "leftHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.76, 6.06, 0.17)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "rightHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.75, 6.43, 0.16)
    step.swing_data[0].profile.height = 0.35
    step.swing_data[0].profile.duration = rospy.Duration(3.0)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "leftFore"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.16, 6.06, 0.111056029797)
    goal.steps.append(step)

    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "rightFore"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.16, 6.43, 0.111056029797)
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "leftHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.55, 6.06, 0.11)
    step.swing_data[0].profile.height = 0.2
    goal.steps.append(step)
    
    stepNumber = stepNumber + 1
    step = deepcopy(step1)
    step.step_number = stepNumber
    step.swing_data[0].name = "rightHind"
    step.swing_data[0].profile.target.point = geometry_msgs.msg.Point(3.55, 6.43, 0.11)
    step.swing_data[0].profile.height = 0.2
    goal.steps.append(step)
    
    
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
