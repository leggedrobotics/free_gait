/******************************************************************************
 * Copyright (C) 2014 by Samuel Bachmann                                      *
 * samuel.bachmann@gmail.com                                                  *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "rqt_free_gait_action/WorkerThreadGetCollections.h"

namespace rqt_free_gait {

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

void WorkerThreadGetCollections::run() {
  CollectionModel *collectionModel = new CollectionModel();
  free_gait_msgs::GetCollectionsRequest collectionsRequest;
  free_gait_msgs::GetCollectionsResponse collectionsResponse;

  if (!ros::service::waitForService(collectionsClient_.getService(),
                                    ros::Duration(600.0))) {
    ROS_WARN_STREAM_NAMED(TAG, TAG
        << " Service: " << collectionsClient_.getService()
        << " is not available. (Timeout 600.0 seconds)");
    emit result(false, collectionModel);
    return;
  }
  if (!collectionsClient_.call(collectionsRequest, collectionsResponse)) {
    emit result(false, collectionModel);
    return;
  }

  std::vector<Action> allActions;
  // Loop over the collections, get their actions and add them finally to the
  // collection model.
  for (auto collectionItem : collectionsResponse.collections) {
    // Get the actions of the collection.
    free_gait_msgs::GetActionsRequest actionsRequest;
    free_gait_msgs::GetActionsResponse actionsResponse;
    actionsRequest.collection_id = collectionItem.id;
    if (!actionsClient_.call(actionsRequest, actionsResponse)) {
      ROS_WARN_STREAM_NAMED(TAG, TAG
          << " Could not get action descriptions for collection id: "
          << collectionItem.id);
      continue;
    }

    // Loop over the actions and add them to the action model.
    ActionModel *actionModel = new ActionModel();
    for (auto actionItem : actionsResponse.actions) {
      Action action(QString::fromStdString(actionItem.id),
                    QString::fromStdString(actionItem.name),
                    QString::fromStdString(actionItem.description));
      actionModel->addAction(action);

      // Check if action is already in the allActions vector.
      bool isFirst = true;
      for (auto allItem : allActions) {
        if (allItem.getId().compare(action.getId()) == 0) {
          isFirst = false;
          break;
        }
      }
      if (isFirst) {
        // Add the actions to the allAction vector.
        allActions.push_back(action);
      }
    }

    // Sort the action names (ASC).
    actionModel->sortActions();

    // Add the action model to the collection.
    Collection collection(QString::fromStdString(collectionItem.id),
                          QString::fromStdString(collectionItem.name),
                          actionModel);

    // Add the collection to the collection model.
    collectionModel->addCollection(collection);
  }

  // Add all actions to the action model.
  ActionModel *actionModel = new ActionModel();
  for (auto item : allActions) {
    actionModel->addAction(item);
  }
  // Add a new collection 'All', with all available actions,
  // to the collection model.
  Collection collection("all", "All", actionModel);
  collectionModel->addCollection(collection);
  // Sort the collection names (ASC).
  // TODO Ensure that 'All' is always on top.
  collectionModel->sortCollections();

  // Return the collection model.
  emit result(true, collectionModel);
}

/*****************************************************************************/
/** Accessors                                                               **/
/*****************************************************************************/

void WorkerThreadGetCollections::setActionsClient(ros::ServiceClient &client) {
  actionsClient_ = client;
}

void WorkerThreadGetCollections::setCollectionsClient(
    ros::ServiceClient &client) {
  collectionsClient_ = client;
}

} // namespace
