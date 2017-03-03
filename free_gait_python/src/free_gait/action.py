#! /usr/bin/env python

import roslib
from free_gait import *
import threading
from actionlib_msgs.msg import GoalStatus
import roslaunch
import tempfile
import os
import traceback


class ActionState:
    ERROR = -1         # Error state.
    UNINITIALIZED = 0  # Not initialized.
    INITIALIZED = 1    # Successfully initialized.
    PENDING = 2        # Waiting for previous action to finish.
    ACTIVE = 3         # Action running.
    IDLE = 4           # Waiting for input.
    DONE = 5           # Finished (success or preempted).

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
    """Base class for generic actions. See specialized implementations below."""

    def __init__(self, relay):
        """Initialization of the action. Implement here subscribers, service
        servers etc."""
        self.state = ActionState.UNINITIALIZED
        self.feedback_callback = None
        self.done_callback = None
        self.relay = relay
        self.goal = None
        self.feedback = None
        self.timeout = rospy.Duration()
        self.set_state(ActionState.INITIALIZED)

    def set_state(self, state):
        """Set the state of the robot and call according callback functions."""
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
        """The action loader registers here its callback methods. These are called
        upon change of state in `set_state(...)`."""
        self.feedback_callback = feedback_callback
        self.done_callback = done_callback

    def start(self):
        """Action is started from the action loader if the actions state is
        `ActionState.INITIALIZED`. Start here the computation of your action."""

    def wait_for_state(self, state):
        """Helper method to wait for a state of the action."""
        wait_for_state = WaitForState(self, state)
        wait_for_state.wait();

    def stop(self):
        """Action is stopped from the action loader. Implement here destruction
        and shutdown procedures."""
        if not self._use_preview():
            self.relay.stop_tracking_goal()
        self.set_state(ActionState.DONE)

    def _send_goal(self):
        """Sends the `self.goal` object to the Free Gait execute steps action server.
        Typically, do not overwrite this method."""
        if self.goal is None:
            self.result = free_gait_msgs.msg.ExecuteStepsResult()
            self.set_state(ActionState.DONE)
            return

        if self._use_preview():
            actionGoal = free_gait_msgs.msg.ExecuteStepsActionGoal()
            actionGoal.goal = self.goal
            self.relay.publish(actionGoal)
            self.set_state(ActionState.ACTIVE)
        else:
            if self.relay.gh:
                self.relay.stop_tracking_goal()
            self.relay.wait_for_server()
            self.relay.send_goal(self.goal,
                                  done_cb=self._done_callback,
                                  active_cb=self._active_callback,
                                  feedback_cb=self._feedback_callback)
            self.set_state(ActionState.PENDING)

    def _active_callback(self):
        """Callback from the execute steps action server when action becomes active
        after it has been sent."""
        self.set_state(ActionState.ACTIVE)

    def _feedback_callback(self, feedback):
        """Feedback callback from the execute steps action server on the progress
        of execution."""
        self.feedback = feedback

    def _done_callback(self, status, result):
        """Done callback from the execute steps action server when action has finished."""
        self.result = result
        if status != GoalStatus.SUCCEEDED \
           and status != GoalStatus.PREEMPTED \
           and status != GoalStatus.RECALLED:
            self.set_state(ActionState.ERROR)
        else:
            self.set_state(ActionState.DONE)

    def _use_preview(self):
        """Helper method to figure out if this actions is run for preview or not."""
        if type(self.relay) == rospy.topics.Publisher:
            return True
        else:
            return False

class SimpleAction(ActionBase):
    """Base class for simple actions with one known goal at initialization."""

    def __init__(self, relay, goal):
        """Initialization of the simple action with storing the goal."""
        ActionBase.__init__(self, relay)
        self.goal = goal

    def start(self):
        """Sends the goal at start to the execute steps action server."""
        ActionBase.start(self)
        self._send_goal()


class ContinuousAction(ActionBase):
    """Base class for actions the run forever if not stopped or preempted."""

    def start(self):
        """Sends the goal at start to the execute steps action server."""
        ActionBase.start(self)
        self._send_goal()

    def _done_callback(self, status, result):
        """Done callback from the execute steps action server when action has finished.
        Insted of switching to state `DONE, continuous actions switch to state `IDLE`."""
        self.result = result
        if status != GoalStatus.SUCCEEDED \
           and status != GoalStatus.PREEMPTED \
           and status != GoalStatus.RECALLED:
            self.set_state(ActionState.ERROR)
        else:
            self.set_state(ActionState.IDLE)


class LaunchAction(ActionBase):

    def __init__(self, file_path, relay):
        ActionBase.__init__(self, relay)
        launch_file = open(file_path, "r")
        launch_text = launch_file.read()
        launch_file.close()
        launch_text = self._replace_preview_argument(launch_text)
        self.temp_launch_file = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
        self.temp_launch_file.write(launch_text)
        self.temp_launch_file.close()
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.listener = roslaunch.pmon.ProcessListener()
        self.listener.process_died = self._process_died
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [self.temp_launch_file.name], process_listeners=[self.listener])

    def start(self):
        try:
            self.launch.start()
        except roslaunch.core.RLException:
            rospy.logerr(traceback.print_exc())
            self.set_state(ActionState.ERROR)
            return

        self.set_state(ActionState.ACTIVE)

    def stop(self):
        self.launch.shutdown()
        os.unlink(self.temp_launch_file.name)
        ActionBase.stop(self)

    def _replace_preview_argument(self, launch_text):
        preview_argument = '<arg name="use_preview" value="'
        if self._use_preview():
            preview_argument += 'true'
        else:
            preview_argument += 'false'
        preview_argument += '"/>'
        new_text = launch_text.replace('<arg name="use_preview" default="true"/>', preview_argument, 1)
        new_text = new_text.replace('<arg name="use_preview" default="false"/>', preview_argument, 1)
        return new_text

    def _process_died(self, process_name, exit_code):
        # For now we assume only one process.
        if exit_code == 0:
            self.set_state(ActionState.DONE)
        else:
            self.set_state(ActionState.ERROR)


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
