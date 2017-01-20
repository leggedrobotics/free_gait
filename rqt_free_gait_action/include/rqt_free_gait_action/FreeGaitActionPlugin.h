/******************************************************************************
 * Copyright 2017 Samuel Bachmann                                             *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions are met:*
 *                                                                            *
 * 1. Redistributions of source code must retain the above copyright notice,  *
 * this list of conditions and the following disclaimer.                      *
 *                                                                            *
 * 2. Redistributions in binary form must reproduce the above copyright       *
 * notice, this list of conditions and the following disclaimer in the        *
 * documentation and/or other materials provided with the distribution.       *
 *                                                                            *
 * 3. Neither the name of the copyright holder nor the names of its           *
 * contributors may be used to endorse or promote products derived from this  *
 * software without specific prior written permission.                        *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE  *
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR        *
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF       *
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS   *
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN    *
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)    *
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF     *
 * THE POSSIBILITY OF SUCH DAMAGE.                                            *
 *                                                                            *
 * Author: Samuel Bachmann <samuel.bachmann@gmail.com>                        *
 ******************************************************************************/

#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <ui_FreeGaitActionPlugin.h>

#include <map>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include <QWidget>
#include <QObject>
#include <QItemSelection>
#include <QStatusBar>
#include <QResizeEvent>
#include <QMenu>

#include "rqt_free_gait_action/ActionModel.h"
#include "rqt_free_gait_action/CollectionModel.h"
#include "rqt_free_gait_action/FavoritePushButton.h"
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

  const QString FAVORITE_INFO = "No favorite collection selected. "
      "Right click on a collection to set as favorite.";

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
  QString favoriteCollectionId_ = "";

  ActionModel *currentActionModel_ = nullptr;
  CollectionModel *collectionModel_ = nullptr;

  bool isCollectionListViewConnected_ = false;
  bool isActionListViewConnected_ = false;
  bool isSendingFavoriteAction_ = false;

  std::vector<FavoritePushButton*> favoritesPushButtons_;

  /***************************************************************************/
  /** Methods                                                               **/
  /***************************************************************************/

  void getCollections();

  void clearActionListView();

  void clearCollectionListView();

  void evaluateFreeGaitActionResponse(
      free_gait_msgs::SendActionResponse response);

  void updateFavoritesInfo(QString info = "");

  void setFavoriteActions(QString collectionId);

  void generateGridLayout();

  void cleanGridLayout(bool deleteWidgets);

  /**
   * @brief Helper function. Removes all layout items within the given @a layout
   * which either span the given @a row or @a column. If @a deleteWidgets
   * is true, all concerned child widgets become not only removed from the
   * layout, but also deleted.
   * https://goo.gl/dObpgP
   * modified to detach widgets
   */
  void remove(QGridLayout *layout, int row, int column, bool deleteWidgets);

  /**
   * @brief Helper function. Deletes all child widgets of the given layout @a
   * item.
   * https://goo.gl/dObpgP
   */
  void deleteChildWidgets(QLayoutItem *item);

  /**
   * @brief Helper function. Detaches all child widgets of the given layout @a
   * item by setting parent to 0.
   */
  void detachChildWidgets(QLayoutItem *item);

  /**
   * @brief Removes all layout items on the given @a row from the given grid
   * @a layout. If @a deleteWidgets is true, all concerned child widgets
   * become not only removed from the layout, but also deleted. Note that
   * this function doesn't actually remove the row itself from the grid
   * layout, as this isn't possible (i.e. the rowCount() and row indices
   * will stay the same after this function has been called).
   * https://goo.gl/dObpgP
   */
  void removeRow(QGridLayout *layout, int row, bool deleteWidgets);

  /**
   * @brief Removes all layout items on the given @a column from the given grid
   * @a layout. If @a deleteWidgets is true, all concerned child widgets
   * become not only removed from the layout, but also deleted. Note that
   * this function doesn't actually remove the column itself from the grid
   * layout, as this isn't possible (i.e. the columnCount() and column
   * indices will stay the same after this function has been called).
   * https://goo.gl/dObpgP
   */
  void removeColumn(QGridLayout *layout, int column, bool deleteWidgets);

  /***************************************************************************/
  /** Events                                                                **/
  /***************************************************************************/

  bool eventFilter(QObject *object, QEvent *event);

protected slots:

  /***************************************************************************/
  /** Slots                                                                 **/
  /***************************************************************************/

  void listViewCollectionsContextMenu(const QPoint &pos);

  void onSendActionClicked();

  void onSendPreviewClicked();

  void onRefreshCollectionsClicked();

  void onFavoriteButtonClicked(Action action);

  void onFavoriteButtonResult(bool isOk,
                              free_gait_msgs::SendActionResponse response);

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

  /***************************************************************************/
  /** Signals                                                               **/
  /***************************************************************************/

  void statusMessage(std::string message, MessageType type,
                     double displaySeconds);

};

} // namespace
