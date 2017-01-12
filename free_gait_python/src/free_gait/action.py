#! /usr/bin/env python

import roslib
from free_gait import *
import threading
from actionlib_msgs.msg import GoalStatus
import roslaunch


class ActionState:
    ERROR = -1
    UNINITIALIZED = 0
    INITIALIZED = 1
    PENDING = 2
    ACTIVE = 3
    IDLE = 4
    DONE = 5


class ActionBase(object):

    def __init__(self, client, directory = None):
        self.state = ActionState.UNINITIALIZED
        self.feedback_callback = None
        self.done_callback = None
        self.client = client
        self.directory = directory
        self.goal = None
        self.feedback = None
        self.result = None
        self.timeout = rospy.Duration()
        self._set_state(ActionState.INITIALIZED)

    def _set_state(self, state):
        self.state = state
        if self.state == ActionState.ERROR \
        or self.state == ActionState.INITIALIZED \
        or self.state == ActionState.PENDING \
        or self.state == ActionState.ACTIVE \
        or self.state == ActionState.IDLE:
            if self.feedback_callback:
                self.feedback_callback()
        elif self.state == ActionState.DONE:
            if self.done_callback:
                self.done_callback()

    def register_callback(self, feedback_callback = None, done_callback = None):
        self.feedback_callback = feedback_callback
        self.done_callback = done_callback

    def start(self):
        self._set_state(ActionState.PENDING)

    def wait_for_result(self):
        wait_for_done = WaitForDone(self)
        wait_for_done.wait();

    def stop(self):
        pass

    def _send_goal(self):
        if self.goal is None:
            self.result = free_gait_msgs.msg.ExecuteStepsResult()
            self.result.status = free_gait_msgs.msg.ExecuteStepsResult.RESULT_UNKNOWN
            self._set_state(ActionState.DONE)
            return

        if self.client.gh:
            self.client.stop_tracking_goal()
        self.client.wait_for_server()
        self.client.send_goal(self.goal,
                              done_cb=self._done_callback,
                              active_cb=self._active_callback,
                              feedback_cb=self._feedback_callback)

    def _active_callback(self):
        self._set_state(ActionState.ACTIVE)

    def _feedback_callback(self, feedback):
        self.feedback = feedback

    def _done_callback(self, status, result):
        self._set_state(ActionState.DONE)
        self.result = result
        if status != GoalStatus.SUCCEEDED:
            self.stop()


class SimpleAction(ActionBase):

    def __init__(self, client, goal):
        ActionBase.__init__(self, client, None)
        self.goal = goal

    def start(self):
        self.state = ActionState.PENDING
        self._send_goal()


class ExternalAction(ActionBase):

    def __init__(self, client, file_path):
        ActionBase.__init__(self, client, None)
        self.file_path = file_path

    def start(self):
        self.state = ActionState.PENDING
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [self.file_path])
        self.launch.start()

    def stop(self):
        self.launch.shutdown()


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
