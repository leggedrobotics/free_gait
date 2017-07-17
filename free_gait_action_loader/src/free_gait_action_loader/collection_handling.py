#! /usr/bin/env python

import os
import rospy
import roslib
import rospkg
from rosparam import load_file
from os.path import *
import free_gait_msgs.msg


class Collection:

    def __init__(self, parameters):
        self.id = None
        self.name = None
        self.action_ids = []
        self.is_sequence = None
        self._initialize(parameters)

    def __str__(self):
        output = 'ID: ' + self.id
        if self.name:
            output = output + ", Name: " + self.name
        if self.action_ids:
            output = output + ", Action IDs: "
            for action_id in self.action_ids:
                output = output + ", " + action_id
        return output

    def _initialize(self, parameters):
        if 'id' in parameters:
            self.id = parameters['id']
        if 'name' in parameters:
            self.name = parameters['name']
        for action_id in parameters['actions']:
            self.action_ids.append(action_id)
        self.is_sequence = False
        if 'is_sequence' in parameters:
            if parameters['is_sequence']:
                self.is_sequence = True
        if not self.is_sequence:
            self.action_ids = sorted(self.action_ids)

    def to_ros_message(self):
        message = free_gait_msgs.msg.CollectionDescription()
        message.id = self.id
        message.name = self.name
        for action_id in self.action_ids:
            message.action_ids.append(action_id)
        message.is_sequence = self.is_sequence
        return message


class CollectionList:

    def __init__(self, name):
        self.name = name
        self.collections = []
        self.collections_to_merge = []

    def update(self):
        self.collections = []
        self.collections_to_merge = []
        rospack = rospkg.RosPack()
        packages = rospack.get_depends_on(self.name, implicit=False)
        for package in packages:
            manifest = rospack.get_manifest(package)
            file_path = manifest.get_export(self.name, 'collections')
            if not file_path:
                continue
            elif len(file_path) != 1:
                rospy.logwarn("Cannot load collections [%s]: invalid 'collections' attribute."%(pkg))
                continue

            file_path = file_path[0]
            try:
                if not os.path.isfile(file_path):
                    rospy.logwarn('Collections parameter file with path "' + file_path + '" does not exists.')
                    continue

                parameters = load_file(file_path)
                for collections_parameters in parameters[0][0]['collections']:
                    if 'collection' in collections_parameters:
                        collection = Collection(collections_parameters['collection'])
                        self.collections.append(collection)
                    elif 'add_to_collection' in collections_parameters:
                        collection = Collection(collections_parameters['add_to_collection'])
                        self.collections_to_merge.append(collection)

            except Exception:
                rospy.logwarn("Unable to load collections [%s] from package [%s]."%(file_path, package))

        self._merge_collections()
        self.collections = sorted(self.collections, key = lambda x: (x.id))
        return True

    def get(self, id):
        collection = [ c for c in self.collections if (c.id == id) ]
        if len(collection) == 0:
            return None
        return collection[0]

    def to_ros_message(self):
        message = []
        for collection in self.collections:
            message.append(collection.to_ros_message())
        return message

    def _merge_collections(self):
        for collection_to_merge in self.collections_to_merge:
            collection = self.get(collection_to_merge.id)
            if not collection:
                rospy.logwarn('Could not find collection with id "%s" to add actions to.'%(collection_to_merge.id))
                continue
            collection.action_ids.extend(collection_to_merge.action_ids)
