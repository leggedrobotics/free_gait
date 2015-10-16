#! /usr/bin/env python

from os import listdir
from os.path import *
import any_msgs.srv
import locomotion_controller_msgs.srv

class Action(ActionBase):
    
    def __init__(self, client, directory):
        ActionBase.__init__(self, client, directory)
        self.actions = [self._create_foot_contact,
                        self._stand_up]
        
    def start(self):
        self._do_next_action()
        
    def wait_for_result(self):
        wait_for_done = WaitForDone(self)
        wait_for_done.wait();
        
    def _done_callback(self, status, result):
        self.result = result
        self._do_next_action()
            
    def _toggle_state_estimator(self, enable):
        service_name = '/locomotion_controller/FreeGaitRos/check_state_estimator'
        rospy.wait_for_service(service_name)
        try:
            check_state_estimator = rospy.ServiceProxy(service_name, any_msgs.srv.Toggle)
            response = check_state_estimator(enable)
            return response.success
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            return False
            
    def _do_next_action(self):
        if self.actions:
            if not self.actions[0]():
                self.state = ActionState.DONE
                self.result = actionlib_msgs.msg.GoalStatus.RESULT_FAILED
            self.actions.pop(0)
        else:
            self.state = ActionState.DONE
        
    # Actions.
    def _create_foot_contact(self):
        service_name = '/locomotion_controller/get_active_controller'
        rospy.wait_for_service(service_name)
        try:
            get_active_controller = rospy.ServiceProxy(service_name, locomotion_controller_msgs.srv.GetActiveController)
            active_controller = get_active_controller()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            return False
        
        controller_name = "FreeGaitRos"
        if active_controller.active_controller_name != controller_name:
            rospy.loginfo('stand_up.py: Activating loco free gait.')
            service_name = '/locomotion_controller/switch_controller'
            rospy.wait_for_service(service_name)
            try:
                switch_controller = rospy.ServiceProxy(service_name, locomotion_controller_msgs.srv.SwitchController)
                switch = switch_controller(controller_name)
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s" % e)
                return False
            
            if switch.status == switch.STATUS_ERROR or switch.status == switch.STATUS_NOTFOUND:
                rospy.logerr('stand_up.py: Error when switching to loco free gait controller.')
                return False
                
        rospy.loginfo('stand_up.py: Deactivating checking for state estimator.')
        if not self._toggle_state_estimator(False):
            return False
        
        rospy.loginfo('stand_up.py: Creating foot contact.')
        self.goal = load_from_file(self.directory + '/stand_up_fragments/create_foot_contact.yaml')
        self._send_goal()
        return True
    
    def _stand_up(self):
        rospy.loginfo('stand_up.py: Activating checking for state estimator.')
        if not self._toggle_state_estimator(True):
            return False
        
        rospy.loginfo('stand_up.py: Standing up.')
        self.goal = load_from_file(self.directory + '/stand_up_fragments/stand_up.yaml')
        self._send_goal()
        return True
        
        
action = Action(action_loader.client, action_loader.directory)
