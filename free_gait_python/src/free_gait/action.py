#! /usr/bin/env python

import roslib
from free_gait import *
import threading
from actionlib_msgs.msg import GoalStatus
import roslaunch


class ActionState:
    ERROR = -1         # Error state.
    UNINITIALIZED = 0  # Not initialized.
    INITIALIZED = 1    # Successfully initialized.
    PENDING = 2        # Waiting for previous action to finish.
    ACTIVE = 3         # Action running.
    IDLE = 4           # Waiting for input.
    DONE = 5           # Successfully finished.

    @staticmethod
    def to_text(action_state):
        if action_state == ActionState.ERROR:
            return 'Error'
        elif action_state == ActionState.UNINITIALIZED:
            return 'Uninitialized'
        elif action_state == ActionState.INITIALIZED:
            return 'Initialized'
        elif action_state == ActionState.PENDING:
            return 'Pending'
        elif action_state == ActionState.ACTIVE:
            return 'Active'
        elif action_state == ActionState.IDLE:
            return 'Idle'
        elif action_state == ActionState.DONE:
            return 'Done'
        else:
            return None


class ActionBase(object):

    def __init__(self, client, directory = None, use_preview = False, preview_publisher = None):
        self.state = ActionState.UNINITIALIZED
        self.feedback_callback = None
        self.done_callback = None
        self.client = client
        self.directory = directory
        self.use_preview = use_preview
        self.preview_publisher = preview_publisher
        self.goal = None
        self.feedback = None
        self.timeout = rospy.Duration()
        self.set_state(ActionState.INITIALIZED)

    def set_state(self, state):
        if state == self.state:
            return
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
        if self.use_preview:
            self.set_state(ActionState.ACTIVE)
        else:
            self.set_state(ActionState.PENDING)

    def wait_for_state(self, state):
        wait_for_state = WaitForState(self, state)
        wait_for_state.wait();

    def stop(self):
        pass

    def _send_goal(self):
        if self.goal is None:
            self.result = free_gait_msgs.msg.ExecuteStepsResult()
            self.result.status = free_gait_msgs.msg.ExecuteStepsResult.RESULT_UNKNOWN
            self.set_state(ActionState.DONE)
            return

        if self.use_preview:
            actionGoal = free_gait_msgs.msg.ExecuteStepsActionGoal()
            actionGoal.goal = self.goal
            self.preview_publisher.publish(actionGoal)
        else:
            if self.client.gh:
                self.client.stop_tracking_goal()
            self.client.wait_for_server()
            self.client.send_goal(self.goal,
                                  done_cb=self._done_callback,
                                  active_cb=self._active_callback,
                                  feedback_cb=self._feedback_callback)

    def _active_callback(self):
        self.set_state(ActionState.ACTIVE)

    def _feedback_callback(self, feedback):
        self.feedback = feedback

    def _done_callback(self, status, result):
        self.set_state(ActionState.DONE)
        self.result = result
        if status != GoalStatus.SUCCEEDED:
            self.stop()


class SimpleAction(ActionBase):

    def __init__(self, client, goal, use_preview = False, preview_publisher = None):
        ActionBase.__init__(self, client, None, use_preview, preview_publisher)
        self.goal = goal

    def start(self):
        ActionBase.start(self)
        self._send_goal()


class ExternalAction(ActionBase):

    def __init__(self, client, file_path, use_preview = False, preview_publisher = None):
        ActionBase.__init__(self, client, None, use_preview, preview_publisher)
        self.file_path = file_path

    def start(self):
        ActionBase.start(self)
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


class WaitForState:

    def __init__(self, action, state, timeout = rospy.Duration(), loop_period = rospy.Duration(0.1)):
        self.action = action
        self.state = state
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
                if not isinstance(self.state, list):
                    if self.action.state == self.state:
                        break
                else:
                    if self.action.state in self.state:
                        break
                if time_left > loop_period or self.timeout == rospy.Duration():
                    time_left = loop_period
                self.done_condition.wait(time_left.to_sec())

        if not isinstance(self.state, list):
            return self.action.state == self.state
        else:
            return self.action.state in self.state
