#! /usr/bin/env python

import roslib
from copy import deepcopy
roslib.load_manifest('loco_free_gait_ros')
import rospy
import actionlib
import starleth_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg

def step_client():
    client = actionlib.SimpleActionClient('/locomotion_controller/step', starleth_msgs.msg.StepAction)
    client.wait_for_server()
    
    goal = starleth_msgs.msg.StepGoal()
    
    step = starleth_msgs.msg.Step()
    step.step_number = 1
    
    # Feet trajectories
    swingData = starleth_msgs.msg.SwingData()
    swingData.name = "leftFore";
    swingData.trajectory.header.frame_id = "base"
    swingData.trajectory.joint_names.append(swingData.name)
    point1 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point1.time_from_start = rospy.Duration(0.0)
    point1.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(0.27456, 0.21753, -0.35885), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point1)
    point2 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point2.time_from_start = rospy.Duration(1.0)
    point2.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(0.27456, 0.18, -0.26), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point2)
    point3 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point3.time_from_start = rospy.Duration(2.0)
    point3.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(0.27456, 0.35, -0.38), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point3)
    point4 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point4.time_from_start = rospy.Duration(3.5)
    point4.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(0.27456, 0.21753, -0.35885), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point4)
    step.swing_data.append(swingData)
    del swingData, point1, point2, point3, point4
    
    swingData = starleth_msgs.msg.SwingData()
    swingData.name = "rightFore";
    swingData.trajectory.header.frame_id = "base"
    swingData.trajectory.joint_names.append(swingData.name)
    point1 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point1.time_from_start = rospy.Duration(0.0)
    point1.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(0.27456, -0.21753, -0.35885), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point1)
    point2 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point2.time_from_start = rospy.Duration(1.0)
    point2.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(0.27456, -0.18, -0.26), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point2)
    point3 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point3.time_from_start = rospy.Duration(2.0)
    point3.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(0.27456, -0.35, -0.38), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point3)
    point4 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point4.time_from_start = rospy.Duration(3.5)
    point4.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(0.27456, -0.21753, -0.35885), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point4)
    step.swing_data.append(swingData)
    del swingData, point1, point2, point3, point4
    
    swingData = starleth_msgs.msg.SwingData()
    swingData.name = "leftHind";
    swingData.trajectory.header.frame_id = "base"
    swingData.trajectory.joint_names.append(swingData.name)
    point1 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point1.time_from_start = rospy.Duration(0.0)
    point1.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(-0.27456, 0.21753, -0.35885), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point1)
    point2 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point2.time_from_start = rospy.Duration(1.0)
    point2.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(-0.27456, 0.18, -0.26), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point2)
    point3 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point3.time_from_start = rospy.Duration(2.0)
    point3.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(-0.27456, 0.35, -0.38), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point3)
    point4 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point4.time_from_start = rospy.Duration(3.5)
    point4.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(-0.27456, 0.21753, -0.35885), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point4)
    step.swing_data.append(swingData)
    del swingData, point1, point2, point3, point4
    
    swingData = starleth_msgs.msg.SwingData()
    swingData.name = "rightHind";
    swingData.trajectory.header.frame_id = "base"
    swingData.trajectory.joint_names.append(swingData.name)
    point1 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point1.time_from_start = rospy.Duration(0.0)
    point1.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(-0.27456, -0.21753, -0.35885), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point1)
    point2 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point2.time_from_start = rospy.Duration(1.0)
    point2.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(-0.27456, -0.18, -0.26), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point2)
    point3 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point3.time_from_start = rospy.Duration(2.0)
    point3.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(-0.27456, -0.35, -0.38), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point3)
    point4 = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
    point4.time_from_start = rospy.Duration(3.5)
    point4.transforms.append(geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(-0.27456, -0.21753, -0.35885), geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    swingData.trajectory.points.append(point4)
    step.swing_data.append(swingData)
    del swingData, point1, point2, point3, point4

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
        rospy.init_node('joint_trajectories_demo')
        result = step_client()
        print "Result:"
        print result
    except rospy.ROSInterruptException:
        print "Program interrupted before completion."