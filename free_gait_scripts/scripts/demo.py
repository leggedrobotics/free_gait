#! /usr/bin/env python

import roslib
from copy import deepcopy
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
    
    # Default step in place
    step = quadruped_msgs.msg.Step()
    step.step_number = 1
    swingData = quadruped_msgs.msg.SwingData()
    swingData.name = "rightFore";
    swingData.profile.target.header.frame_id = "map"
    swingData.profile.target.point = geometry_msgs.msg.Point(0.279204123175, -0.2189, 0.02491)
    step.swing_data.append(swingData)
    goal.steps.append(step)
    del step, swingData
    
    # High, slow step
    step = quadruped_msgs.msg.Step()
    step.step_number = 2
    swingData = quadruped_msgs.msg.SwingData()
    swingData.name = "rightHind";
    swingData.profile.target.header.frame_id = "map"
    swingData.profile.target.point = geometry_msgs.msg.Point(-0.279204123175, -0.2189, 0.02491)
    swingData.profile.height = 0.4
    swingData.profile.duration = rospy.Duration(5.0)
    step.swing_data.append(swingData)
    goal.steps.append(step)
    del step, swingData
    
    # Foot trajectory
    step = quadruped_msgs.msg.Step()
    step.step_number = 3
    swingData = quadruped_msgs.msg.SwingData()
    swingData.name = "leftFore";
    swingData.trajectory.header.frame_id = "map"
    swingData.trajectory.joint_names.append(swingData.name)
    point1 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point1.time_from_start = rospy.Duration(0.0)
    point1.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(0.279204123175, 0.2189, 0.02491), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point1)
    point2 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point2.time_from_start = rospy.Duration(1.0)
    point2.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(0.2, 0.15, 0.2), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point2)
    point3 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point3.time_from_start = rospy.Duration(2.0)
    point3.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(0.35, 0.25, 0.2), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point3)
    point4 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point4.time_from_start = rospy.Duration(3.0)
    point4.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(0.279204123175, 0.2189, 0.02491), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point4)
    step.swing_data.append(swingData)
    goal.steps.append(step)
    del step, swingData, point1, point2, point3, point4
    
#     # High, slow step
#     step2 = quadruped_msgs.msg.StepData()
#     step2.step_number = 2
#     step2.leg_names.append("rightHind")
#     step2.swing_durations.append(rospy.Duration(3.5))
#     step2.positions.append(geometry_msgs.msg.Point(-0.279204123175, -0.2189, 0.02491))
#     step2.normals.append(geometry_msgs.msg.Vector3(0.0, 0.0, 1.0))
#     step2.swing_heights.append(0.4)
#     goal.steps.append(step2)
#     
#     # Slow and fast shift
#     step3 = quadruped_msgs.msg.StepData()
#     step3.step_number = 3
#     step3.leg_names.append("leftFore")
#     step3.swing_durations.append(rospy.Duration(1.5))
#     step3.positions.append(geometry_msgs.msg.Point(0.279204123175, 0.2189, 0.02491))
#     step3.normals.append(geometry_msgs.msg.Vector3(0.0, 0.0, 1.0))
#     step3.swing_heights.append(0.2)
#     step3.state_names.append("pre_step")
#     step3.shift_durations.append(rospy.Duration(3.0))
#     step3.state_names.append("post_step")
#     step3.shift_durations.append(rospy.Duration(0.2))
#     goal.steps.append(step3)
#     
#     # Trot
#     step4 = quadruped_msgs.msg.StepData()
#     step4.step_number = 1
#     step4.leg_names.append("rightHind")
#     step4.leg_names.append("leftFore")
#     step4.swing_durations.append(rospy.Duration(0.3))
#     step4.swing_durations.append(step4.swing_durations[0])
#     step4.positions.append(geometry_msgs.msg.Point(-0.279, -0.218, 0.0249))
#     step4.positions.append(geometry_msgs.msg.Point(0.279, 0.218, 0.0249))
#     step4.normals.append(geometry_msgs.msg.Vector3(0.0, 0.0, 1.0))
#     step4.normals.append(step4.normals[0])
#     step4.swing_heights.append(0.12)
#     step4.swing_heights.append(step1.swing_heights[0])
#     step4.state_names.append("pre_step")
#     step4.state_names.append("post_step")
#     step4.shift_durations.append(rospy.Duration(1.5))
#     step4.shift_durations.append(step4.shift_durations[0])
#     goal.steps.append(step4)
#     
#     # Asymmetric trot
#     step4 = quadruped_msgs.msg.StepData()
#     step4.step_number = 1
#     step4.leg_names.append("leftHind")
#     step4.leg_names.append("rightFore")
#     step4.swing_durations.append(rospy.Duration(0.2))
#     step4.swing_durations.append(rospy.Duration(0.7))
#     step4.positions.append(geometry_msgs.msg.Point(-0.279, 0.218, 0.0249))
#     step4.positions.append(geometry_msgs.msg.Point(0.279, -0.218, 0.0249))
#     step4.normals.append(geometry_msgs.msg.Vector3(0.0, 0.0, 1.0))
#     step4.normals.append(step4.normals[0])
#     step4.swing_heights.append(0.09)
#     step4.swing_heights.append(0.2)
#     step4.state_names.append("pre_step")
#     step4.state_names.append("post_step")
#     step4.shift_durations.append(rospy.Duration(1.5))
#     step4.shift_durations.append(step4.shift_durations[0])
#     goal.steps.append(step4)
    
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
        rospy.init_node('free_gait_demo')
        result = step_client()
        print "Result:"
        print result
    except rospy.ROSInterruptException:
        print "Program interrupted before completion."
