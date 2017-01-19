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

#include "rqt_free_gait_action/FreeGaitActionPlugin.h"

#include <pluginlib/class_list_macros.h>

namespace rqt_free_gait {

/*****************************************************************************/
/** Constructor/Destructor                                                  **/
/*****************************************************************************/

FreeGaitActionPlugin::FreeGaitActionPlugin()
    : rqt_gui_cpp::Plugin()
    , widget_(0) {

  qRegisterMetaType<free_gait_msgs::SendActionResponse>(
      "free_gait_msgs::SendActionResponse");
  qRegisterMetaType<std_srvs::TriggerResponse>(
      "std_srvs::TriggerResponse");

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

  // Reset labels.
  ui_.labelActionDescription->setText("");

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

  // Get collections.
  getCollections();
}

void FreeGaitActionPlugin::shutdownPlugin() {
  collectionsClient_.shutdown();
  actionsClient_.shutdown();
  sendActionClient_.shutdown();
  sendPreviewClient_.shutdown();
  refreshCollectionsClient_.shutdown();
}

/*****************************************************************************/
/** Settings                                                                **/
/*****************************************************************************/

void FreeGaitActionPlugin::saveSettings(
    qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const {
}

void FreeGaitActionPlugin::restoreSettings(
    const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings) {
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
  } else {
    currentActionModel_ = collectionModel_->getActionModel(
        selection.indexes().first());
    ui_.listViewActions->setModel(currentActionModel_);
    ui_.labelActionDescription->setText("");

    connect(ui_.listViewActions->selectionModel(),
            SIGNAL(selectionChanged(QItemSelection, QItemSelection)),
            this, SLOT(onActionSelectionChanged(QItemSelection)));
    isActionListViewConnected_ = true;
  }
}

void FreeGaitActionPlugin::onActionSelectionChanged(
    const QItemSelection &selection) {
  if (selection.indexes().isEmpty()) {
    ui_.labelActionDescription->setText("");
  } else {
    if (currentActionModel_ == nullptr) {
      return;
    }
    Action action = currentActionModel_->getAction(selection.indexes().first());
    ui_.labelActionDescription->setText(action.getDescription());
    selectedAction_ = action.getId();
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
  } else {
    // TODO clean collection and action list view?
  }
}

void FreeGaitActionPlugin::onSendActionResult(
    bool isOk, free_gait_msgs::SendActionResponse response) {
  if (!isOk) {
    emit statusMessage("Could not send action.", MessageType::WARNING, 2.0);
  } else {
    switch (response.result.status) {
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
  ui_.pushButtonSend->setEnabled(true);
}

void FreeGaitActionPlugin::onSendActionClicked() {
  if (selectedAction_.isEmpty()) {
    emit statusMessage("No action selected.", MessageType::WARNING, 2.0);
    return;
  }

  free_gait_msgs::SendActionRequest request;
  request.goal.action_id = selectedAction_.toStdString();

  WorkerThreadSendAction *workerThreadSendAction = new WorkerThreadSendAction;
  connect(workerThreadSendAction,
          SIGNAL(result(bool, free_gait_msgs::SendActionResponse)),
          this,
          SLOT(onSendActionResult(bool, free_gait_msgs::SendActionResponse)));
  connect(workerThreadSendAction, SIGNAL(finished()),
          workerThreadSendAction, SLOT(deleteLater()));
  workerThreadSendAction->setClient(sendActionClient_);
  workerThreadSendAction->setRequest(request);
  workerThreadSendAction->start();

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
          SIGNAL(result(bool, free_gait_msgs::SendActionResponse)),
          this,
          SLOT(onSendPreviewResult(bool, free_gait_msgs::SendActionResponse)));
  connect(workerThreadSendPreview, SIGNAL(finished()),
          workerThreadSendPreview, SLOT(deleteLater()));
  workerThreadSendPreview->setClient(sendPreviewClient_);
  workerThreadSendPreview->setRequest(request);
  workerThreadSendPreview->start();

  ui_.pushButtonPreview->setEnabled(false);
}

void FreeGaitActionPlugin::onSendPreviewResult(
    bool isOk, free_gait_msgs::SendActionResponse response) {
  if (!isOk) {
    emit statusMessage("Could not send preview action.",
                       MessageType::WARNING, 2.0);
  } else {
    switch (response.result.status) {
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

} // namespace

PLUGINLIB_EXPORT_CLASS(rqt_free_gait::FreeGaitActionPlugin, rqt_gui_cpp::Plugin)

