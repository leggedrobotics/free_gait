#! /usr/bin/env python

class Action(ActionBase):

    def __init__(self, client):
        ActionBase.__init__(self, client)
        self.goal = load_from_file('/home/peter/catkin_ws/src/free_gait/free_gait_python/actions/starleth/trot_once.yaml', 'map')
        self.trigger = TriggerOnFeedback(1, 0.3)
        self.timeout = rospy.Duration(10.0)
        
    def _feedback_callback(self, feedback):
        ActionBase._feedback_callback(self, feedback)
        if self.trigger.check(self.feedback):
            self._send_goal()
            
    def _done_callback(self, status, result):
        pass
            
    def wait_for_result(self):
        wait_for_done = WaitForDone(self, self.timeout)
        wait_for_done.wait();

        
action = Action(action_loader.client)
