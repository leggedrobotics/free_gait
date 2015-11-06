#! /usr/bin/env python

class Action(ActionBase):
    
    def __init__(self, client, directory):
        ActionBase.__init__(self, client, directory)
        self.goal = free_gait_msgs.msg.ExecuteStepsGoal()
        
        step = free_gait_msgs.msg.Step()
        base_auto = free_gait_msgs.msg.BaseAuto()
        base_auto.height = 0.45
        base_auto.average_linear_velocity = 0.05
        base_auto.average_angular_velocity = 0.2
        base_auto.support_margin = 0.05
        step.base_auto.append(base_auto)
        self.goal.steps.append(step)
        del step, base_auto
        
        step = free_gait_msgs.msg.Step()
        footstep = free_gait_msgs.msg.Footstep()
        footstep.name = 'leftFore'
        footstep.target.header.frame_id = 'map'
        footstep.target.point.x = 0.35
        footstep.target.point.y = 0.2
        footstep.target.point.z = 0.0
        step.footstep.append(footstep)
        self.goal.steps.append(step)
        del step, footstep
        
        step = free_gait_msgs.msg.Step()
        base_auto = free_gait_msgs.msg.BaseAuto()
        step.base_auto.append(base_auto)
        self.goal.steps.append(step)
        del step, base_auto

action = Action(action_loader.client, action_loader.directory)
