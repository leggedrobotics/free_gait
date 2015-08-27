import free_gait_msgs.msg
from free_gait import *
    
def get_foot_position_for_angle(foot_name, angle):
    # Rotates the step position around the current base center by an angle.
    import geometry_msgs.msg
    from math import cos, sin

    length = 0.2792
    width = 0.2189
    height = 0.0
    position = geometry_msgs.msg.Point();
    
    if foot_name == 'rightFore':
        width = -width
    elif foot_name == 'leftHind':
        length = -length
    elif foot_name == 'rightHind':
        width = -width
        length = -length
        
    position.x = length * cos(angle) - width * sin(angle)
    position.y = length * sin(angle) + width * cos(angle)
    position.z = height
    return position;


goal = free_gait_msgs.msg.StepGoal()
angle = 0.35
    
for cycle in range(0, 30):
    step = free_gait_msgs.msg.Step()
    step.step_number = 4 * cycle + 1
    swingData = free_gait_msgs.msg.SwingData()
    swingData.name = 'rightFore';
    swingData.profile.target.header.frame_id = 'map'
    swingData.profile.target.point = get_foot_position_for_angle(swingData.name, cycle * angle)
    step.swing_data.append(swingData)
    goal.steps.append(step)
    del step, swingData
    
    step = free_gait_msgs.msg.Step()
    step.step_number = 4 * cycle + 2
    swingData = free_gait_msgs.msg.SwingData()
    swingData.name = 'leftFore';
    swingData.profile.target.header.frame_id = 'map'
    swingData.profile.target.point = get_foot_position_for_angle(swingData.name, cycle * angle)
    step.swing_data.append(swingData)
    goal.steps.append(step)
    del step, swingData
    
    step = free_gait_msgs.msg.Step()
    step.step_number = 4 * cycle + 3
    swingData = free_gait_msgs.msg.SwingData()
    swingData.name = 'leftHind';
    swingData.profile.target.header.frame_id = 'map'
    swingData.profile.target.point = get_foot_position_for_angle(swingData.name, cycle * angle)
    step.swing_data.append(swingData)
    goal.steps.append(step)
    del step, swingData
    
    step = free_gait_msgs.msg.Step()
    step.step_number = 4 * cycle + 4
    swingData = free_gait_msgs.msg.SwingData()
    swingData.name = 'rightHind';
    swingData.profile.target.header.frame_id = 'map'
    swingData.profile.target.point = get_foot_position_for_angle(swingData.name, cycle * angle)
    step.swing_data.append(swingData)
    goal.steps.append(step)
    del step, swingData
    
(translation, rotation) = transform_coordinates('map', 'footprint')
adapt_coordinates(goal, translation, rotation)
