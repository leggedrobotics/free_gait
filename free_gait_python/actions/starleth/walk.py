#! /usr/bin/env python

# Action that generates footholds based on the current desired
# velocities.

import tf
from numpy import *
from copy import deepcopy

class Leg:
    LF = 0
    RF = 1
    LH = 2
    RH = 3
    
    @staticmethod
    def to_text(leg):
        if leg == Leg.LF:
            return 'leftFore'
        elif leg == Leg.RF:
            return 'rightFore'
        elif leg == Leg.LH:
            return 'leftHind'
        elif leg == Leg.RH:
            return 'rightHind'
        else:
            return None

class Direction:
    FORWARD = 0
    BACKWARD = 1
    LEFT = 2
    RIGHT = 3
        
class Action(ActionBase):

    def __init__(self, client):
            
        ActionBase.__init__(self, client)
        self.trigger = TriggerOnFeedback(1, 0.5)
        self.timeout = rospy.Duration(10)
        self.keep_alive = True
        
        self.velocity = array([1, 0])
        self.direction = None
        self._determine_direction()
        self.leg = None

        self.step_length_factor = array([0.25, 0.1])
        self.default_foot_position = dict()
        self.default_foot_position[Leg.LF]  = array([ 0.2525,  0.185])
        self.default_foot_position[Leg.RF] = array([ 0.2525, -0.185])
        self.default_foot_position[Leg.LH]  = array([-0.2525,  0.185])
        self.default_foot_position[Leg.RH] = array([-0.2525, -0.185])
        self.joy_velocity_frame = "footprint"
        self.step_frame = "map"
        
        self.gait_pattern = dict()
        self.gait_pattern[Direction.FORWARD] = [Leg.RH, Leg.RF, Leg.LH, Leg.LF]
        self.gait_pattern[Direction.BACKWARD] = deepcopy(self.gait_pattern[Direction.FORWARD])
        self.gait_pattern[Direction.BACKWARD].reverse()
        self.gait_pattern[Direction.RIGHT] = [Leg.LH, Leg.RH, Leg.LF, Leg.RF]
        self.gait_pattern[Direction.LEFT] = deepcopy(self.gait_pattern[Direction.RIGHT])
        self.gait_pattern[Direction.LEFT].reverse()
        
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1.0) # fix for tf bug
        
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
        self._select_leg();
        next_foothold_in_footprint = append(self.default_foot_position[self.leg] + self.step_length_factor * self.velocity, 0.0)
        next_foothold, _ = transform_coordinates(self.joy_velocity_frame, self.step_frame, next_foothold_in_footprint, listener = self.tf_listener) # in map frame
        
        self.goal = free_gait_msgs.msg.StepGoal()
        step = free_gait_msgs.msg.Step()
        step.step_number = 1
        swing_data = free_gait_msgs.msg.SwingData()
        swing_data.name = Leg.to_text(self.leg);
        swing_data.profile.target.header.frame_id = self.step_frame
        swing_data.profile.target.point = geometry_msgs.msg.Point(next_foothold[0], next_foothold[1], next_foothold[2])
        step.swing_data.append(swing_data)
        self.goal.steps.append(step)
        
    def _determine_direction(self):
        if abs(self.velocity[0]) >= abs(self.velocity[1]):
            if self.velocity[0] >= 0:
                self.direction = Direction.FORWARD
            else:
                self.direction = Direction.BACKWARD
        else:
            if self.velocity[1] >= 0:
                self.direction = Direction.LEFT
            else:
                self.direction = Direction.RIGHT
                
    def _select_leg(self):
        if self.direction is None:
            self.leg = None
            return
        
        if self.leg is None:
            self.leg = self.gait_pattern[self.direction][0]
            return
        
        index = self.gait_pattern[self.direction].index(self.leg)
        index = index + 1
        if index >= len(self.gait_pattern[self.direction]):
            index = 0
        
        self.leg = self.gait_pattern[self.direction][index]
        

action = Action(action_loader.client)
