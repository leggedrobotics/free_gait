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

#include <std_srvs/Trigger.h>

namespace rqt_free_gait {

class WorkerThreadUpdateTrigger : public QThread {
Q_OBJECT

  /***************************************************************************/
  /** Methods                                                               **/
  /***************************************************************************/

  void run();

public:

  /***************************************************************************/
  /** Accessors                                                             **/
  /***************************************************************************/

  void setClient(ros::ServiceClient &client);

private:

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  std_srvs::TriggerRequest request_;
  std_srvs::TriggerResponse response_;
  ros::ServiceClient client_;

signals:

  /***************************************************************************/
  /** Signals                                                               **/
  /***************************************************************************/

  void result(bool isOk, std_srvs::TriggerResponse response);
};

} // namespace
