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

#pragma once

#include <QThread>

#include <ros/ros.h>

#include <free_gait_msgs/GetActions.h>
#include <free_gait_msgs/GetCollections.h>

#include "rqt_free_gait_action/CollectionModel.h"

namespace rqt_free_gait {

class WorkerThreadGetCollections : public QThread {
Q_OBJECT

  /***************************************************************************/
  /** Methods                                                               **/
  /***************************************************************************/

  void run();

public:

  /***************************************************************************/
  /** Accessors                                                             **/
  /***************************************************************************/

  void setActionsClient(ros::ServiceClient &client);

  void setCollectionsClient(ros::ServiceClient &client);

private:

  /***************************************************************************/
  /** Constants                                                             **/
  /***************************************************************************/

  const std::string TAG = "WorkerThreadGetCollections";

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  ros::ServiceClient actionsClient_;
  ros::ServiceClient collectionsClient_;

signals:

  /***************************************************************************/
  /** Signals                                                               **/
  /***************************************************************************/

  void result(bool isOk, CollectionModel *collectionModel);
};

} // namespace
