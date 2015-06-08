#! /usr/bin/env python

import rospy
import quadruped_msgs.msg
from tf.transformations import *
import geometry_msgs.msg

def load_from_file(file_path, position = [0, 0, 0], orientation = [0, 0, 0, 1]):
    from rosparam import load_file
    import os
    
    if not os.path.isfile(file_path):
        rospy.logerr('File with path "' + file_path + '" does not exists.')
        return

    return get_from_yaml(load_file(file_path), position, orientation)


def get_from_yaml(yaml_object, position = [0, 0, 0], orientation = [0, 0, 0, 1]):
    goal = quadruped_msgs.msg.StepGoal()
    step_number = 0
    
    # For each step.
    for step_parameter in yaml_object[0][0]['steps']:

        step_parameter = step_parameter['step']
        # Step.
        step = quadruped_msgs.msg.Step()
        
        # Step number
        step_number = step_number + 1
        step.step_number = step_number
        
        # Swing data.
        if 'swing_data' in step_parameter:
            for swing_data_parameter in step_parameter['swing_data']:
                swing_data = quadruped_msgs.msg.SwingData()
                
                # Name.
                swing_data.name = swing_data_parameter['name']
            
                # Profile.
                if 'profile' in swing_data_parameter:
                    # Profile target.
                    target_parameter = swing_data_parameter['profile']['target']
                    swing_data.profile.target.point.x = target_parameter[0]
                    swing_data.profile.target.point.y = target_parameter[1]
                    swing_data.profile.target.point.z = target_parameter[2]
                    # Profile target frame.
                    if 'target_frame' in swing_data_parameter['profile']:
                        swing_data.profile.target.header.frame_id = swing_data_parameter['profile']['target_frame']
                    # Profile height.
                    if 'height' in swing_data_parameter['profile']:
                        swing_data.profile.height = swing_data_parameter['profile']['height']
                    # Profile duration.
                    if 'duration' in swing_data_parameter['profile']:
                        swing_data.profile.duration = rospy.Duration(swing_data_parameter['profile']['duration'])
                    # Profile type.
                    if 'type' in swing_data_parameter['profile']:
                        swing_data.profile.type = swing_data_parameter['profile']['type']
                
                # Expect touchdown.
                if 'expect_touchdown' in swing_data_parameter:
                    swing_data.expect_touchdown = swing_data_parameter['expect_touchdown']
                # Surface normal.
                if 'surface_normal' in swing_data_parameter:
                    normal_parameter = swing_data_parameter['surface_normal']
                    swing_data.surface_normal.vector.x = normal_parameter[0]
                    swing_data.surface_normal.vector.y = normal_parameter[1]
                    swing_data.surface_normal.vector.z = normal_parameter[2] 
                
                step.swing_data.append(swing_data)
    
        # Base shift data.
        if 'base_shift_data' in step_parameter:
            for base_shift_data_parameter in step_parameter['base_shift_data']:
                
                base_shift_data = quadruped_msgs.msg.BaseShiftData()
                
                # Name.
                base_shift_data.name = base_shift_data_parameter['name']
            
                # Profile.
                if 'profile' in base_shift_data_parameter:
                    # Profile target.
                    if 'target' in base_shift_data_parameter['profile']:
                        target_parameter = base_shift_data_parameter['profile']['target']
                        base_shift_data.profile.target.point.x = target_parameter[0]
                        base_shift_data.profile.target.point.y = target_parameter[1]
                        base_shift_data.profile.target.point.z = target_parameter[2]
                    # Profile target frame.
                    if 'target_frame' in base_shift_data_parameter['profile']:
                        base_shift_data.profile.target.header.frame_id = base_shift_data_parameter['profile']['target_frame']
                    # Profile height.
                    if 'height' in base_shift_data_parameter['profile']:
                        base_shift_data.profile.height = base_shift_data_parameter['profile']['height']
                    # Profile duration.
                    if 'duration' in base_shift_data_parameter['profile']:
                        base_shift_data.profile.duration = rospy.Duration(base_shift_data_parameter['profile']['duration'])
                    # Profile type.
                    if 'type' in base_shift_data_parameter['profile']:
                        base_shift_data.profile.type = base_shift_data_parameter['profile']['type']

                step.base_shift_data.append(base_shift_data)
                
        goal.steps.append(step)
    
    # Adapt to local coordinates if desired.
    if 'adapt_to_current_position' in yaml_object[0][0]:
        if yaml_object[0][0]['adapt_to_current_position']:
            adapt_coordinates(goal, position, orientation)
            
    return goal


def adapt_coordinates(goal, position, orientation):
    # For each steps.
    translation = translation_matrix(position)
    z_axis = [0, 0, 1]
    (roll, pitch, yaw) = euler_from_quaternion(orientation)
    rotation = rotation_matrix(yaw, z_axis)
    transform = concatenate_matrices(translation, rotation)
    
    for step in goal.steps:
        for swing_data in step.swing_data:
            swing_data.profile.target.point = transform_point(transform, swing_data.profile.target.point)
            

def transform_point(transform, point):
    transformed_point = transform.dot([point.x, point.y, point.z, 1.0])
    return geometry_msgs.msg.Point(transformed_point[0], transformed_point[1], transformed_point[2])
