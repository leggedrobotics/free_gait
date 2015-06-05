#! /usr/bin/env python

import rospy
import quadruped_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

class ActionConverter:
    'Instantiates a Free Gait action goal from a YAML file.'
    def __init__(self):
        pass
   
    @staticmethod
    def load_from_file(file_path):
        print 'Loading free gait action from "' + file_path + '".'
        from rosparam import load_file
        action = load_file(file_path)
        
        goal = quadruped_msgs.msg.StepGoal()
        
        # For each steps.
        for step_parameter in action[0][0]['steps']:

            # Step.
            step = quadruped_msgs.msg.Step()
            step.step_number = step_parameter['step_number']
            
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
        
#             # Base shift data.
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
        
        return goal