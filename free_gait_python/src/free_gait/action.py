#! /usr/bin/env python

import roslib
from free_gait import *
import threading
from actionlib_msgs.msg import GoalStatus
import roslaunch


class ActionState:
    UNINITIALIZED = 0
    INITIALIZED = 1
    PENDING = 2
    ACTIVE = 3
    DONE = 4


class ActionBase(object):

    def __init__(self, client, directory = None):
        self.state = ActionState.UNINITIALIZED
        self.client = client
        self.directory = directory
        self.goal = None
        self.feedback = None
        self.result = None
        self.timeout = rospy.Duration()
        # If true, action can run in background after state DONE.
        self.keep_alive = False
        self.state = ActionState.INITIALIZED

    def start(self):
        self.state = ActionState.PENDING
        self._send_goal()

    def wait_for_result(self):
        wait_for_done = WaitForDone(self)
        wait_for_done.wait();

    def stop(self):
        pass

    def _send_goal(self):
        if self.goal is None:
            self.result = free_gait_msgs.msg.ExecuteStepsResult()
            self.result.status = free_gait_msgs.msg.ExecuteStepsResult.RESULT_UNKNOWN
            self.state = ActionState.DONE
            return

        if self.client.gh:
            self.client.stop_tracking_goal()
        self.client.wait_for_server()
        self.client.send_goal(self.goal,
                              done_cb=self._done_callback,
                              active_cb=self._active_callback,
                              feedback_cb=self._feedback_callback)

    def _active_callback(self):
        self.state = ActionState.ACTIVE

    def _feedback_callback(self, feedback):
        self.feedback = feedback

    def _done_callback(self, status, result):
        self.state = ActionState.DONE
        self.result = result
        if status != GoalStatus.SUCCEEDED:
            self.stop()


class SimpleAction(ActionBase):

    def __init__(self, client, goal):
        ActionBase.__init__(self, client, None)
        self.goal = goal


class ContinuousAction(ActionBase):

    def __init__(self, client, directory = None):
        ActionBase.__init__(self, client, directory)
        self.keep_alive = True

    def start(self):
        self.state = ActionState.PENDING

    def wait_for_result(self):
        # Immediate return because action runs in background.
        self.result = free_gait_msgs.msg.ExecuteStepsResult()
        self.result.status = self.result.RESULT_UNKNOWN


class ExternalAction(ActionBase):

    def __init__(self, client, file_path):
        ActionBase.__init__(self, client, None)
        self.file_path = file_path
        self.keep_alive = True

    def start(self):
        self.state = ActionState.PENDING
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [self.file_path])
        self.launch.start()

    def wait_for_result(self):
        # Immediate return because action runs externally.
        self.result = free_gait_msgs.msg.ExecuteStepsResult()
        self.result.status = self.result.RESULT_UNKNOWN

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
