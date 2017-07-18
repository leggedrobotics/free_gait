/******************************************************************************
 * Copyright 2017 Samuel Bachmann, Peter Fankhauser                           *
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
 * Authors: Samuel Bachmann <samuel.bachmann@gmail.com>,                      *
 *          Peter Fankhauser <pfankhauser@ethz.ch>                            *
 ******************************************************************************/

#include "rqt_free_gait_action/FreeGaitActionPlugin.h"

#include <pluginlib/class_list_macros.h>

namespace rqt_free_gait {

/*****************************************************************************/
/** Constructor/Destructor                                                  **/
/*****************************************************************************/

FreeGaitActionPlugin::FreeGaitActionPlugin()
    : rqt_gui_cpp::Plugin()
    , widget_(0) {

  qRegisterMetaType<free_gait_msgs::ExecuteActionResult>(
      "free_gait_msgs::ExecuteActionResult");
  qRegisterMetaType<std_srvs::TriggerResponse>(
      "std_srvs::TriggerResponse");
  qRegisterMetaType<Action>("Action");

  setObjectName("FreeGaitActionPlugin");
}

/*****************************************************************************/
/** Initialize/Shutdown                                                     **/
/*****************************************************************************/

void FreeGaitActionPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(widget_->windowTitle() +
        " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // Install event filter.
  ui_.widgetFavorites->installEventFilter(this);

  // Reset labels.
  ui_.labelActionDescription->setText("");

  // Set up custom context menu for collections list view.
  ui_.listViewCollections->setContextMenuPolicy(Qt::CustomContextMenu);

  // Initialize status bar.
  statusBar_ = new QStatusBar(widget_);
  statusBar_->setSizeGripEnabled(false);
  statusBar_->setStyleSheet("border: none");
  statusBar_->show();
  ui_.statusBarLayout->addWidget(statusBar_);

  // Initialize service clients.
  collectionsClient_ = getNodeHandle().serviceClient<
      free_gait_msgs::GetCollections>(
      "/free_gait_action_loader/list_collections", false);
  actionsClient_ = getNodeHandle().serviceClient<
      free_gait_msgs::GetActions>(
      "/free_gait_action_loader/list_actions", false);
  sendActionClient_ = getNodeHandle().serviceClient<
      free_gait_msgs::SendAction>(
      "/free_gait_action_loader/send_action", false);
  sendPreviewClient_ = getNodeHandle().serviceClient<
      free_gait_msgs::SendAction>(
      "/free_gait_action_loader/preview_action", false);
  sendActionSequenceClient_ = getNodeHandle().serviceClient<
      free_gait_msgs::SendActionSequence>(
      "/free_gait_action_loader/send_action_sequence", false);
  refreshCollectionsClient_ = getNodeHandle().serviceClient<
      std_srvs::Trigger>(
      "/free_gait_action_loader/update", false);

  // Connect signals to slots.
  connect(ui_.pushButtonSend, SIGNAL(clicked()),
          this, SLOT(onSendActionClicked()));
  connect(ui_.pushButtonPreview, SIGNAL(clicked()),
          this, SLOT(onSendPreviewClicked()));
  connect(ui_.pushButtonRefreshCollections, SIGNAL(clicked()),
          this, SLOT(onRefreshCollectionsClicked()));
  connect(this, SIGNAL(statusMessage(std::string, MessageType, double)),
          this, SLOT(onStatusMessage(std::string, MessageType, double)));
  connect(ui_.listViewCollections,
          SIGNAL(customContextMenuRequested(const QPoint&)),
          this, SLOT(listViewCollectionsContextMenu(const QPoint&)));

  // Initialize favorites info.
  updateFavoritesInfo(FAVORITE_INFO);

  // Get collections.
  getCollections();
}

void FreeGaitActionPlugin::shutdownPlugin() {
  collectionsClient_.shutdown();
  actionsClient_.shutdown();
  sendActionClient_.shutdown();
  sendPreviewClient_.shutdown();
  sendActionSequenceClient_.shutdown();
  refreshCollectionsClient_.shutdown();
}

/*****************************************************************************/
/** Settings                                                                **/
/*****************************************************************************/

void FreeGaitActionPlugin::saveSettings(
    qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const {
  plugin_settings.setValue("favorite_collection_id", favoriteCollectionId_);
}

void FreeGaitActionPlugin::restoreSettings(
    const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings) {
  favoriteCollectionId_ =
      plugin_settings.value("favorite_collection_id", "").toString();
}

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

void FreeGaitActionPlugin::getCollections() {
  WorkerThreadGetCollections *workerThreadGetCollections =
      new WorkerThreadGetCollections;
  connect(workerThreadGetCollections, SIGNAL(result(bool, CollectionModel*)),
          this, SLOT(onGetCollectionsResult(bool, CollectionModel*)));
  connect(workerThreadGetCollections, SIGNAL(finished()),
          workerThreadGetCollections, SLOT(deleteLater()));
  workerThreadGetCollections->setActionsClient(actionsClient_);
  workerThreadGetCollections->setCollectionsClient(collectionsClient_);
  workerThreadGetCollections->start();
}

void FreeGaitActionPlugin::clearActionListView() {
  ActionModel *actionModel = new ActionModel(this);
  ui_.listViewActions->setModel(actionModel);
}

void FreeGaitActionPlugin::clearCollectionListView() {
  if (collectionModel_ != nullptr) {
    delete collectionModel_;
  }
  collectionModel_ = new CollectionModel(this);
  ui_.listViewCollections->setModel(collectionModel_);
}

void FreeGaitActionPlugin::evaluateFreeGaitActionResponse(
    free_gait_msgs::ExecuteActionResult result) {
  switch (result.status) {
    case free_gait_msgs::ExecuteActionResult::RESULT_DONE:
      emit statusMessage("Done", MessageType::SUCCESS, 2.0);
      break;
    case free_gait_msgs::ExecuteActionResult::RESULT_STARTED :
      emit statusMessage("Started", MessageType::SUCCESS, 2.0);
      break;
    case free_gait_msgs::ExecuteActionResult::RESULT_FAILED:
      emit statusMessage("Failed", MessageType::ERROR, 2.0);
      break;
    case free_gait_msgs::ExecuteActionResult::RESULT_NOT_FOUND:
      emit statusMessage("Not found", MessageType::ERROR, 2.0);
      break;
    case free_gait_msgs::ExecuteActionResult::RESULT_UNKNOWN:
      emit statusMessage("Unknown", MessageType::WARNING, 2.0);
      break;
    default:
      break;
  }
}

void FreeGaitActionPlugin::updateFavoritesInfo(QString info) {
  ui_.labelFavoritesInfo->setText(info);
  if (ui_.gridLayoutFavorites->count() > 0) {
    ui_.labelFavoritesInfo->hide();
  } else {
    ui_.labelFavoritesInfo->show();
  }
}

void FreeGaitActionPlugin::setFavoriteActions(QString collectionId) {
  if (isSendingFavoriteAction_) {
    statusMessage("Cannot change favorites while sending a favorite action.",
                  MessageType::WARNING, 2.0);
    return;
  }
  cleanGridLayout(true);
  favoritesPushButtons_.clear();
  if (collectionId.isEmpty()) {
    updateFavoritesInfo(FAVORITE_INFO);
    return;
  }
  // Set up favorites.
  for (auto item : collectionModel_->getActions(collectionId)) {
    favoritesPushButtons_.push_back(new FavoritePushButton(item));
    connect(favoritesPushButtons_.back(), SIGNAL(clicked(Action)),
            this, SLOT(onFavoriteButtonClicked(Action)));
  }
  generateGridLayout();
  if (favoritesPushButtons_.empty()) {
    favoriteCollectionId_ = "";
    updateFavoritesInfo(FAVORITE_INFO);
  } else {
    updateFavoritesInfo();
  }
}

void FreeGaitActionPlugin::remove(QGridLayout *layout, int row,
                                  int column, bool deleteWidgets) {
  // We avoid usage of QGridLayout::itemAtPosition() here
  // to improve performance.
  for (int i = layout->count() - 1; i >= 0; i--) {
    int r, c, rs, cs;
    layout->getItemPosition(i, &r, &c, &rs, &cs);
    if ((r <= row && r + rs - 1 >= row) ||
        (c <= column && c + cs - 1 >= column)) {
      // This layout item is subject to deletion.
      QLayoutItem *item = layout->takeAt(i);
      if (deleteWidgets) {
        deleteChildWidgets(item);
      } else {
        detachChildWidgets(item);
      }
      delete item;
    }
  }
}

void FreeGaitActionPlugin::deleteChildWidgets(QLayoutItem *item) {
  if (item->layout()) {
    // Process all child items recursively.
    for (int i = 0; i < item->layout()->count(); i++) {
      deleteChildWidgets(item->layout()->itemAt(i));
    }
  }
  delete item->widget();
}

void FreeGaitActionPlugin::detachChildWidgets(QLayoutItem *item) {
  if (item->layout()) {
    // Process all child items recursively.
    for (int i = 0; i < item->layout()->count(); i++) {
      detachChildWidgets(item->layout()->itemAt(i));
    }
  } else {
    item->widget()->setParent(0);
  }
}

void FreeGaitActionPlugin::removeRow(QGridLayout *layout,
                                     int row, bool deleteWidgets) {
  remove(layout, row, -1, deleteWidgets);
  layout->setRowMinimumHeight(row, 0);
  layout->setRowStretch(row, 0);
}

void FreeGaitActionPlugin::removeColumn(QGridLayout *layout, int column,
                                        bool deleteWidgets) {
  remove(layout, -1, column, deleteWidgets);
  layout->setColumnMinimumWidth(column, 0);
  layout->setColumnStretch(column, 0);
}

void FreeGaitActionPlugin::cleanGridLayout(bool deleteWidgets) {
  // Remove all widgets from grid layout.
  if (ui_.gridLayoutFavorites->rowCount() > 0) {
    for (int i = 0; i < ui_.gridLayoutFavorites->rowCount(); ++i) {
      removeRow(ui_.gridLayoutFavorites, i, deleteWidgets);
    }
    ui_.gridLayoutFavorites->invalidate();
  }
}

void FreeGaitActionPlugin::generateGridLayout() {
  // Set the used font.
  QFont myFont("Ubuntu", 11);
  QFontMetrics fontMetrics(myFont);

  // Get available widget size.
  int width = ui_.widgetFavorites->size().width();

  int numberOfComponents = (int)favoritesPushButtons_.size();

  // Check if it is necessary to regenerate the grid.
  bool isNew = false;
  if (numberOfComponents == favoritesPushButtonsLast_.size()) {
    for (int i = 0; i < numberOfComponents; ++i) {
      if (favoritesPushButtons_[i] != favoritesPushButtonsLast_[i]) {
        isNew = true;
      }
    }
  } else {
    isNew = true;
  }

  if (favoritesPushButtons_.empty()) {
    // No buttons, clean layout and delete all buttons.
    cleanGridLayout(true);
    numberOfColumnsLast_ = 0;
  } else {
    // Compute the minimum width of the buttons, depending on the button text.
    int minWidth = 0;
    for (auto item : favoritesPushButtons_) {
      int fontWidth = fontMetrics.width(item->text()) + 20;
      if (fontWidth > minWidth) {
        minWidth = fontWidth;
      }
    }
    // Compute the number of columns that is possible with the given minimum
    // width.
    int numberOfColumns = 1;
    while (true) {
      // TODO 4 ???
      if (numberOfColumns * minWidth +
          (numberOfColumns - 1) * ui_.gridLayoutFavorites->horizontalSpacing() +
          4 < width) {
        numberOfColumns++;
      } else {
        numberOfColumns--;
        break;
      }
      if (numberOfColumns == favoritesPushButtons_.size() &&
          numberOfColumns * minWidth +
          (numberOfColumns - 1) * ui_.gridLayoutFavorites->horizontalSpacing() +
          4 < width) {
        break;
      }
    }
    if (numberOfColumns < 1) {
      numberOfColumns = 1;
    }

    // Check if and how to clean the grid layout (delete or not delete the
    // buttons.
    if (numberOfColumns != numberOfColumnsLast_ && !isNew) {
      cleanGridLayout(false);
    } else if (isNew) {
      cleanGridLayout(true);
    }

    // If new or the number of columns changed.
    if (numberOfColumns != numberOfColumnsLast_ || isNew) {
      // Add widgets to the grid layout.
      int numberOfComponents = (int)favoritesPushButtons_.size();
      int row = 0;
      unsigned long counter = 0;
      while (numberOfComponents > 0) {
        for (int i = 0; i < numberOfColumns; ++i) {
          if (numberOfComponents > 0) {
            // Add push button to grid layout.
            ui_.gridLayoutFavorites->addWidget(
                favoritesPushButtons_.at(counter), row, i);
            counter++;
            numberOfComponents--;
          }
        }
        row++;
      }
    }
    // Set current number of columns as last.
    numberOfColumnsLast_ = numberOfColumns;
  }
  // Set current set of favorite buttons as last.
  favoritesPushButtonsLast_ = favoritesPushButtons_;
}

/*****************************************************************************/
/** Events                                                                  **/
/*****************************************************************************/

bool FreeGaitActionPlugin::eventFilter(QObject *object, QEvent *event) {
  if (event->type() == QEvent::Resize) {
    QResizeEvent *resizeEvent = static_cast<QResizeEvent *>(event);
    if (resizeEvent->size().width() != resizeEvent->oldSize().width()) {
      generateGridLayout();
    }
  }
  return QObject::eventFilter(object, event);
}

/*****************************************************************************/
/** Slots                                                                   **/
/*****************************************************************************/

void
FreeGaitActionPlugin::onCollectionSelectionChanged(
    const QItemSelection &selection) {
  if (currentActionModel_ != nullptr && isActionListViewConnected_) {
    disconnect(ui_.listViewActions->selectionModel(),
               SIGNAL(selectionChanged(QItemSelection, QItemSelection)),
               this, SLOT(onActionSelectionChanged(QItemSelection)));
    isActionListViewConnected_ = false;
  }
  currentActionModel_ = nullptr;
  if (selection.indexes().isEmpty()) {
    clearActionListView();
    ui_.pushButtonSend->setText("Send Sequence");
    ui_.pushButtonSend->setEnabled(false);
    ui_.pushButtonPreview->setEnabled(false);
  } else {
    currentActionModel_ = collectionModel_->getActionModel(
        selection.indexes().first());
    ui_.listViewActions->setModel(currentActionModel_);
    ui_.labelActionDescription->setText("");

    connect(ui_.listViewActions->selectionModel(),
            SIGNAL(selectionChanged(QItemSelection, QItemSelection)),
            this, SLOT(onActionSelectionChanged(QItemSelection)));
    isActionListViewConnected_ = true;
    selectedAction_ = "";
    selectedCollection_ = "";
    if (collectionModel_->isSequence(selection.indexes().first())) {
      selectedCollection_ = collectionModel_->getCollectionId(selection.indexes().first());
      ui_.pushButtonSend->setEnabled(true);
      ui_.pushButtonSend->setText("Send Sequence");
      ui_.pushButtonPreview->setEnabled(false);
    } else {
      ui_.pushButtonSend->setEnabled(false);
      ui_.pushButtonPreview->setEnabled(false);
    }
  }
}

void FreeGaitActionPlugin::onActionSelectionChanged(
    const QItemSelection &selection) {
  if (selection.indexes().isEmpty()) {
    ui_.labelActionDescription->setText("");
    ui_.pushButtonSend->setText("Send");
    ui_.pushButtonSend->setEnabled(false);
    ui_.pushButtonPreview->setEnabled(false);
  } else {
    if (currentActionModel_ == nullptr) {
      return;
    }
    Action action = currentActionModel_->getAction(selection.indexes().first());
    ui_.labelActionDescription->setText(action.getDescription());
    selectedAction_ = action.getId();
    selectedCollection_ = "";
    ui_.pushButtonSend->setEnabled(true);
    ui_.pushButtonSend->setText("Send");
    ui_.pushButtonPreview->setEnabled(true);
  }
}

void FreeGaitActionPlugin::onGetCollectionsResult(
    bool isOk, CollectionModel *collectionModel) {
  if (isCollectionListViewConnected_) {
    disconnect(ui_.listViewCollections->selectionModel(),
               SIGNAL(selectionChanged(QItemSelection, QItemSelection)),
               this, SLOT(onCollectionSelectionChanged(QItemSelection)));
    isCollectionListViewConnected_ = false;
  }
  if (isOk) {
    collectionModel_ = collectionModel;
    ui_.listViewCollections->setModel(collectionModel_);

    connect(ui_.listViewCollections->selectionModel(),
            SIGNAL(selectionChanged(QItemSelection, QItemSelection)),
            this, SLOT(onCollectionSelectionChanged(QItemSelection)));
    isCollectionListViewConnected_ = true;

    setFavoriteActions(favoriteCollectionId_);
  } else {
    // TODO clean collection and action list view?
  }
}

void FreeGaitActionPlugin::onSendActionResult(
    bool isOk, free_gait_msgs::ExecuteActionResult response) {
  if (!isOk) {
    emit statusMessage("Could not send action.", MessageType::WARNING, 2.0);
  } else {
    evaluateFreeGaitActionResponse(response);
  }
  ui_.pushButtonSend->setEnabled(true);
}

void FreeGaitActionPlugin::onSendActionClicked() {
  if (selectedAction_.isEmpty() && selectedCollection_.isEmpty()) {
    emit statusMessage("Nothing selected.", MessageType::WARNING, 2.0);
    return;
  }

  if (!selectedAction_.isEmpty()) {
    free_gait_msgs::SendActionRequest request;
    request.goal.action_id = selectedAction_.toStdString();

    WorkerThreadSendAction *workerThreadSendAction = new WorkerThreadSendAction;
    connect(workerThreadSendAction, SIGNAL(result(bool, free_gait_msgs::ExecuteActionResult)), this,
            SLOT(onSendActionResult(bool, free_gait_msgs::ExecuteActionResult)));
    connect(workerThreadSendAction, SIGNAL(finished()), workerThreadSendAction, SLOT(deleteLater()));
    workerThreadSendAction->setClient(sendActionClient_);
    workerThreadSendAction->setRequest(request);
    workerThreadSendAction->start();
  } else if (!selectedCollection_.isEmpty()) {
    free_gait_msgs::SendActionSequenceRequest request;
    for (auto item : collectionModel_->getActions(selectedCollection_)) {
      free_gait_msgs::SendActionRequest::_goal_type goal;
      goal.action_id = item.getId().toStdString();
      request.goals.push_back(goal);
    }

    WorkerThreadSendActionSequence *workerThreadSendActionSequence = new WorkerThreadSendActionSequence;
    connect(workerThreadSendActionSequence, SIGNAL(result(bool, free_gait_msgs::ExecuteActionResult)),
            this, SLOT(onSendActionResult(bool, free_gait_msgs::ExecuteActionResult)));
    connect(workerThreadSendActionSequence, SIGNAL(finished()),
            workerThreadSendActionSequence, SLOT(deleteLater()));
    workerThreadSendActionSequence->setClient(sendActionSequenceClient_);
    workerThreadSendActionSequence->setRequest(request);
    workerThreadSendActionSequence->start();
  }

  ui_.pushButtonSend->setEnabled(false);
}

void FreeGaitActionPlugin::onSendPreviewClicked() {
  if (selectedAction_.isEmpty()) {
    emit statusMessage("No action selected.", MessageType::WARNING, 2.0);
    return;
  }

  free_gait_msgs::SendActionRequest request;
  request.goal.action_id = selectedAction_.toStdString();

  WorkerThreadSendAction *workerThreadSendPreview = new WorkerThreadSendAction;
  connect(workerThreadSendPreview,
          SIGNAL(result(bool, free_gait_msgs::ExecuteActionResult)),
          this,
          SLOT(onSendPreviewResult(bool, free_gait_msgs::ExecuteActionResult)));
  connect(workerThreadSendPreview, SIGNAL(finished()),
          workerThreadSendPreview, SLOT(deleteLater()));
  workerThreadSendPreview->setClient(sendPreviewClient_);
  workerThreadSendPreview->setRequest(request);
  workerThreadSendPreview->start();

  ui_.pushButtonPreview->setEnabled(false);
}

void FreeGaitActionPlugin::onSendPreviewResult(
    bool isOk, free_gait_msgs::ExecuteActionResult response) {
  if (!isOk) {
    emit statusMessage("Could not send preview action.",
                       MessageType::WARNING, 2.0);
  } else {
    evaluateFreeGaitActionResponse(response);
  }
  ui_.pushButtonPreview->setEnabled(true);
}

void FreeGaitActionPlugin::onRefreshCollectionsClicked() {
  clearActionListView();
  clearCollectionListView();
  ui_.pushButtonRefreshCollections->setEnabled(false);

  WorkerThreadUpdateTrigger *workerThreadUpdateTrigger =
      new WorkerThreadUpdateTrigger;
  connect(workerThreadUpdateTrigger,
          SIGNAL(result(bool, std_srvs::TriggerResponse)),
          this,
          SLOT(onRefreshCollectionsResult(bool, std_srvs::TriggerResponse)));
  connect(workerThreadUpdateTrigger, SIGNAL(finished()),
          workerThreadUpdateTrigger, SLOT(deleteLater()));
  workerThreadUpdateTrigger->setClient(refreshCollectionsClient_);
  workerThreadUpdateTrigger->start();
}

void FreeGaitActionPlugin::onRefreshCollectionsResult(
    bool isOk, std_srvs::TriggerResponse response) {
  if (!isOk) {
    emit statusMessage("Could not refresh collections.",
                       MessageType::WARNING, 2.0);
  } else {
    emit statusMessage("Collections refreshed.", MessageType::SUCCESS, 2.0);
  }

  ui_.pushButtonRefreshCollections->setEnabled(true);

  getCollections();
}

void FreeGaitActionPlugin::onStatusMessage(std::string message,
                                           MessageType type,
                                           double displaySeconds) {
  switch(type) {
    case MessageType::ERROR:
      statusBar_->setStyleSheet("border: none; color: red");
      ROS_ERROR_STREAM_NAMED(TAG, TAG << " " << message.c_str());
      break;
    case MessageType::WARNING:
      statusBar_->setStyleSheet("border: none; color: orange");
      ROS_WARN_STREAM_NAMED(TAG, TAG << " " << message.c_str());
      break;
    case MessageType::SUCCESS:
      statusBar_->setStyleSheet("border: none; color: green");
      ROS_INFO_STREAM_NAMED(TAG, TAG << " " << message.c_str());
      break;
    case MessageType::STATUS:
      statusBar_->setStyleSheet("border: none; color: black");
      ROS_INFO_STREAM_NAMED(TAG, TAG << " " << message.c_str());
      break;
  }
  statusBar_->showMessage(QString::fromStdString(message),
                          (int)(displaySeconds * 1000));
}

void FreeGaitActionPlugin::listViewCollectionsContextMenu(const QPoint &pos) {
  QPoint globalPos = ui_.listViewCollections->mapToGlobal(pos);

  QModelIndex index = ui_.listViewCollections->indexAt(pos);

  if (index.isValid()) {
    QString collectionId = collectionModel_->getCollectionId(index);

    QMenu menuForItem;
    QAction* actionSetAsFavorite = menuForItem.addAction("Set as favorite");
    QAction* actionResult;
    actionResult = menuForItem.exec(globalPos);
    if(actionResult) {
      if(actionResult == actionSetAsFavorite) {
        favoriteCollectionId_ = collectionId;
        setFavoriteActions(favoriteCollectionId_);
      }
    }
  }
}

void FreeGaitActionPlugin::onFavoriteButtonClicked(Action action) {
  isSendingFavoriteAction_ = true;

  free_gait_msgs::SendActionRequest request;
  request.goal.action_id = action.getId().toStdString();

  WorkerThreadSendAction *workerThreadSendAction = new WorkerThreadSendAction;
  connect(workerThreadSendAction,
          SIGNAL(result(bool, free_gait_msgs::ExecuteActionResult)),
          this,
          SLOT(onFavoriteButtonResult(bool,
                                      free_gait_msgs::ExecuteActionResult)));
  connect(workerThreadSendAction, SIGNAL(finished()),
          workerThreadSendAction, SLOT(deleteLater()));
  workerThreadSendAction->setClient(sendActionClient_);
  workerThreadSendAction->setRequest(request);
  workerThreadSendAction->start();

  for (auto button : favoritesPushButtons_) {
    button->setEnabled(false);
  }
}

void FreeGaitActionPlugin::onFavoriteButtonResult(
    bool isOk, free_gait_msgs::ExecuteActionResult response) {
  if (!isOk) {
    emit statusMessage("Could not send action.", MessageType::WARNING, 2.0);
  } else {
    evaluateFreeGaitActionResponse(response);
  }
  for (auto button : favoritesPushButtons_) {
    button->setEnabled(true);
  }

  isSendingFavoriteAction_ = false;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(rqt_free_gait::FreeGaitActionPlugin, rqt_gui_cpp::Plugin)

