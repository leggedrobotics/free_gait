#! /usr/bin/env python

# Action that generates footholds based on the current desired
# velocities.

import tf
from numpy import *

class Action(ActionBase):

    def __init__(self, client):
            
        ActionBase.__init__(self, client)
        self.trigger = TriggerOnFeedback(1, 0.5)
        self.timeout = rospy.Duration(10)
        self.keep_alive = True
        self.desired_velocity = [0, 0]
        
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1.0) # fix for tf bug

        
        # Parameters
        self.default_foot_position = dict()
        self.default_foot_position['leftFore']  = array([ 0.2525,  0.185])
        self.default_foot_position['rightFore'] = array([ 0.2525, -0.185])
        self.default_foot_position['leftHind']  = array([-0.2525,  0.185])
        self.default_foot_position['rightHind'] = array([-0.2525, -0.185])
        self.joy_velocity_frame = "footprint"
        self.step_frame = "map"
        
        self._generate_goal()

    def _active_callback(self):
        # Immediate return because action runs in background.
        self.state = ActionState.DONE
        self.result = free_gait_msgs.msg.StepResult()
        self.result.status = self.result.RESULT_UNKNOWN
        
    def _feedback_callback(self, feedback):
        ActionBase._feedback_callback(self, feedback)
        if self.trigger.check(self.feedback):
            self._generate_goal()
            self.send_goal()
            
    def _done_callback(self, status, result):
        pass
            
    def wait_for_result(self):
        wait_for_done = WaitForDone(self)
        wait_for_done.wait();
        
    def _generate_goal(self):
#         self.goal = load_from_file('/home/peter/catkin_ws/src/free_gait/free_gait_python/actions/starleth/trot_once.yaml', 'map')
        
        leg = 'rightHind'
        next_foothold_in_footprint = append(self.default_foot_position[leg] + [0.1, 0], 0.0)
        next_foothold, _ = transform_coordinates(self.joy_velocity_frame, self.step_frame, next_foothold_in_footprint, listener = self.tf_listener) # in map frame
        print next_foothold
        
        self.goal = free_gait_msgs.msg.StepGoal()
        step = free_gait_msgs.msg.Step()
        step.step_number = 1
        swing_data = free_gait_msgs.msg.SwingData()
        swing_data.name = leg;
        swing_data.profile.target.header.frame_id = self.step_frame
        swing_data.profile.target.point = geometry_msgs.msg.Point(next_foothold[0], next_foothold[1], next_foothold[2])
        step.swing_data.append(swing_data)
        self.goal.steps.append(step)

action = Action(action_loader.client)
