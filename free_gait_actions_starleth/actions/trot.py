import free_gait_msgs.msg
import geometry_msgs.msg
from free_gait import *

goal = free_gait_msgs.msg.StepGoal()
    
step = free_gait_msgs.msg.Step()
step.step_number = 1
swingData = free_gait_msgs.msg.SwingData()
swingData.name = 'rightFore';
swingData.profile.target.header.frame_id = 'map'
swingData.profile.target.point = geometry_msgs.msg.Point(0.279204123175, -0.2189, 0.0)
step.swing_data.append(swingData)
goal.steps.append(step)
del step, swingData


    
(translation, rotation) = transform_coordinates('map', 'footprint')
adapt_coordinates(goal, translation, rotation)
trigger = TriggerOnFeedback(1, 0.3)