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

action = Action(action_loader.client, action_loader.directory)
