#! /usr/bin/env python

class Action(ActionBase):
    
    def __init__(self, client):
        ActionBase.__init__(self, client)
        self._generate_goal()

    def say_hello(self):
        print "########################################"
        print rospy.get_name()
        print "########################################"
        
    def _generate_goal(self):
        self.say_hello()
        self.goal = free_gait_msgs.msg.StepGoal()
        step = free_gait_msgs.msg.Step()
        step.step_number = 1
        swingData = free_gait_msgs.msg.SwingData()
        swingData.name = 'rightFore';
        swingData.profile.target.header.frame_id = 'map'
        swingData.profile.target.point = geometry_msgs.msg.Point(0.279204123175, -0.2189, 0.0)
        step.swing_data.append(swingData)
        self.goal.steps.append(step)
        (translation, rotation) = transform_coordinates('map', 'footprint')
        adapt_coordinates(self.goal, translation, rotation)
        
    def feedback_callback(self, feedback):
        ActionBase.feedback_callback(self, feedback)
        print self.feedback

action = Action(action_loader.client)