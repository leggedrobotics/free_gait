#! /usr/bin/env python

import roslib
from free_gait import *
import threading

class ActionState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2

class ActionBase(object):
    
    def __init__(self, client, directory = None):
        self.state = ActionState.DONE
        self.client = client
        self.directory = directory
        self.goal = None
        self.feedback = None
        self.result = None
        self.timeout = rospy.Duration()
        # If true, action can run in background after state DONE.
        self.keep_alive = False
    
    def start(self):
        self._send_goal()
        
    def wait_for_result(self):
        self.client.wait_for_result(self.timeout)
        
    def get_result(self):
        return self.result
    
    def _send_goal(self):
        if self.goal is None:
            self.result = free_gait_msgs.msg.ExecuteStepsResult()
            self.result.status = free_gait_msgs.msg.ExecuteStepsResult.RESULT_UNKNOWN
            self.state = ActionState.DONE
            return
        
        self.state = ActionState.PENDING
        if self.client.gh:
            self.client.stop_tracking_goal()
        self.client.wait_for_server()
        self.client.send_goal(self.goal,
                              done_cb = self._done_callback,
                              active_cb = self._active_callback,
                              feedback_cb = self._feedback_callback)

    def _active_callback(self):
        self.state = ActionState.ACTIVE

    def _feedback_callback(self, feedback):
        self.feedback = feedback
        
    def _done_callback(self, status, result):
        self.state = ActionState.DONE
        self.result = result
        
        
class SimpleAction(ActionBase):
    
    def __init__(self, client, goal):
        ActionBase.__init__(self, client, None)
        self.goal = goal
        
    
class TriggerOnFeedback:
    
    def __init__(self, n_steps_in_queue, phase_of_step):
        self.n_steps_in_queue = n_steps_in_queue
        self.phase_of_step = phase_of_step
        self.feedback = None
        
    def check(self, feedback):
        self.feedback = feedback
        if self.feedback.queue_size <= self.n_steps_in_queue and self.feedback.phase >= self.phase_of_step:
            return True
        else:
            return False
        
        
class WaitForDone:
    
    def __init__(self, action, timeout = rospy.Duration(), loop_period = rospy.Duration(0.1)):
        self.action = action
        self.timeout = timeout
        self.loop_period = loop_period
        self.done_condition = threading.Condition()
        
    def wait(self):
        timeout_time = rospy.get_rostime() + self.timeout
        loop_period = rospy.Duration(0.1)
        with self.done_condition:
            while not rospy.is_shutdown():
                time_left = timeout_time - rospy.get_rostime()
                if self.timeout > rospy.Duration(0.0) and time_left <= rospy.Duration(0.0):
                    break
    
                if self.action.state == ActionState.DONE:
                    break
                
                if time_left > loop_period or self.timeout == rospy.Duration():
                    time_left = loop_period
    
                self.done_condition.wait(time_left.to_sec())

        return self.action.state == ActionState.DONE