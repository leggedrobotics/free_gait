#! /usr/bin/env python

import os
import rospy
import roslib
import rospkg
from rosparam import load_file
from os.path import *
import free_gait_msgs.msg

class ActionType:
    YAML = 0
    PYTHON = 1
    LAUNCH = 2

    @staticmethod
    def to_text(action_type):
        if action_type == ActionType.YAML:
            return 'yaml'
        elif action_type == ActionType.PYTHON:
            return 'python'
        elif action_type == ActionType.LAUNCH:
            return 'launch'
        else:
            return None

    @staticmethod
    def from_text(action_type):
        if action_type == 'yaml':
            return ActionType.YAML
        elif action_type == 'python':
            return ActionType.PYTHON
        elif action_type == 'launch':
            return ActionType.LAUNCH
        else:
            return None


class ActionEntry:

    def __init__(self, package_path, parameters):
        self.id = None
        self.name = None
        self.file = None
        self.type = None
        self.description = None
        self.directory = None
        self._initialize(package_path, parameters);

    def __str__(self):
        output = 'ID: ' + self.id
        if self.name:
            output = output + ", Name: " + self.name
        if self.file:
            output = output + ", File: " + self.file
        if self.file:
            output = output + ", Type: " + ActionType.to_text(self.type)
        return output

    def _initialize(self, package_path, parameters):
        if 'id' in parameters:
            self.id = parameters['id']
        if 'name' in parameters:
            self.name = parameters['name']
        if 'file' in parameters:
            self.file = abspath(join(package_path, parameters['file']))
        if 'type' in parameters:
            self.type = ActionType.from_text(parameters['type'])
        if 'description' in parameters:
            self.description = parameters['description']
        self.directory = dirname(abspath(self.file))

    def to_ros_message(self):
        message = free_gait_msgs.msg.ActionDescription()
        message.id = self.id
        message.name = self.name
        message.file = self.file
        message.type =  ActionType.to_text(self.type)
        message.description = self.description
        return message


class ActionList:

    def __init__(self, name):
        self.name = name
        self.actions = []

    def update(self):
        self.actions = []
        rospack = rospkg.RosPack()
        packages = rospack.get_depends_on(self.name, implicit=False)
        for package in packages:
            manifest = rospack.get_manifest(package)
            file_path = manifest.get_export(self.name, 'actions')
            if not file_path:
                continue
            elif len(file_path) != 1:
                rospy.logwarn("Cannot load actions [%s]: invalid 'actions' attribute."%(pkg))
                continue

            file_path = file_path[0]
            try:
                if not os.path.isfile(file_path):
                    rospy.logwarn('Actions parameter file with path "' + file_path + '" does not exists.')
                    continue

                package_path = rospack.get_path(package)
                parameters = load_file(file_path)
                for action_parameters in parameters[0][0]['actions']:
                    entry = ActionEntry(package_path, action_parameters['action'])
                    self.actions.append(entry)

            except Exception:
                rospy.logwarn("Unable to load actions [%s] from package [%s]."%(file_path, package))

        self.actions = sorted(self.actions, key = lambda x: (x.id))
        return True

    def get(self, id):
        entry = [ e for e in self.actions if (e.id == id) ]
        if len(entry) == 0:
            return None
        return entry[0]

    def get_multiple(self, ids):
        entries = []
        for id in ids:
            entry = self.get(id)
            if entry:
                entries.append(entry)
        return entries

    def to_ros_message(self, ids = []):
        actions = []
        if len(ids):
            actions = self.get_multiple(ids)
        else:
            actions = self.actions
        message = []
        for action in actions:
            message.append(action.to_ros_message())
        return message
