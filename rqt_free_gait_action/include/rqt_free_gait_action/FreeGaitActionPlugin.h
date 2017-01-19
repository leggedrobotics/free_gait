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

#include <rqt_gui_cpp/plugin.h>
#include <ui_FreeGaitActionPlugin.h>

#include <string>

#include <ros/ros.h>

#include <QWidget>
#include <QObject>
#include <QItemSelection>
#include <QStatusBar>

#include "rqt_free_gait_action/ActionModel.h"
#include "rqt_free_gait_action/CollectionModel.h"
#include "rqt_free_gait_action/WorkerThreadGetCollections.h"
#include "rqt_free_gait_action/WorkerThreadSendAction.h"
#include "rqt_free_gait_action/WorkerThreadUpdateTrigger.h"

namespace rqt_free_gait {

class FreeGaitActionPlugin : public rqt_gui_cpp::Plugin {
  Q_OBJECT
public:

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  enum class MessageType {
    ERROR,
    WARNING,
    SUCCESS,
    STATUS
  };

  /***************************************************************************/
  /** Constructor/Destructor                                                **/
  /***************************************************************************/

  FreeGaitActionPlugin();

  /***************************************************************************/
  /** Initialize/Shutdown                                                   **/
  /***************************************************************************/

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual void shutdownPlugin();

  /***************************************************************************/
  /** Settings                                                              **/
  /***************************************************************************/

  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                            qt_gui_cpp::Settings& instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                               const qt_gui_cpp::Settings& instance_settings);

protected:

  /***************************************************************************/
  /** Constants                                                             **/
  /***************************************************************************/

  const std::string TAG = "FreeGaitActionPlugin";

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  Ui::FreeGaitActionPluginWidget ui_;
  QWidget* widget_;
  QStatusBar* statusBar_;

  ros::ServiceClient actionsClient_;
  ros::ServiceClient collectionsClient_;
  ros::ServiceClient sendActionClient_;
  ros::ServiceClient sendPreviewClient_;
  ros::ServiceClient refreshCollectionsClient_;

  QString selectedAction_ = "";

  ActionModel *currentActionModel_ = nullptr;
  CollectionModel *collectionModel_ = nullptr;

  bool isCollectionListViewConnected_ = false;
  bool isActionListViewConnected_ = false;

  /***************************************************************************/
  /** Methods                                                               **/
  /***************************************************************************/

  void getCollections();

  void clearActionListView();

  void clearCollectionListView();

protected slots:

  /***************************************************************************/
  /** Slots                                                                 **/
  /***************************************************************************/

  void onSendActionClicked();

  void onSendPreviewClicked();

  void onRefreshCollectionsClicked();

  void onCollectionSelectionChanged(const QItemSelection &selection);

  void onActionSelectionChanged(const QItemSelection &selection);

  void onGetCollectionsResult(bool isOk, CollectionModel *collectionModel);

  void onSendActionResult(bool isOk,
                          free_gait_msgs::SendActionResponse response);

  void onSendPreviewResult(bool isOk,
                           free_gait_msgs::SendActionResponse response);

  void onRefreshCollectionsResult(bool isOk,
                                  std_srvs::TriggerResponse response);

  void onStatusMessage(std::string message, MessageType type,
                       double displaySeconds);

signals:

  /*****************************************************************************/
  /** Signals                                                                 **/
  /*****************************************************************************/

  void statusMessage(std::string message, MessageType type,
                     double displaySeconds);

};

} // namespace
