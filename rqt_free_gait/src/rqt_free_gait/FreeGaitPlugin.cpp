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

#include "rqt_free_gait/FreeGaitPlugin.h"

#include <pluginlib/class_list_macros.h>

namespace rqt_free_gait {

/*****************************************************************************/
/** Constructor/Destructor                                                  **/
/*****************************************************************************/

FreeGaitPlugin::FreeGaitPlugin()
    : rqt_gui_cpp::Plugin(), widget_(0), descriptions_(1000) {

  setObjectName("FreeGaitPlugin");

  qRegisterMetaType<free_gait_msgs::ExecuteStepsActionGoal>
      ("free_gait_msgs::ExecuteStepsActionGoal");
  qRegisterMetaType<free_gait_msgs::ExecuteStepsActionFeedback>
      ("free_gait_msgs::ExecuteStepsActionFeedback");
  qRegisterMetaType<free_gait_msgs::ExecuteStepsActionResult>
      ("free_gait_msgs::ExecuteStepsActionResult");
  qRegisterMetaType<std_srvs::SetBoolResponse>
      ("std_srvs::SetBoolResponse");
  qRegisterMetaType<std_srvs::TriggerResponse>
      ("std_srvs::TriggerResponse");
}

/*****************************************************************************/
/** Initialization/Shutdown                                                 **/
/*****************************************************************************/

void FreeGaitPlugin::initPlugin(qt_gui_cpp::PluginContext &context) {
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(widget_->windowTitle() +
        " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // Initialize subscribers.
  goalSubscriber_ = getNodeHandle().subscribe<
      free_gait_msgs::ExecuteStepsActionGoal>(
      getNodeHandle().param<std::string>("/free_gait/action_server", "") +
          "/goal", 10, &FreeGaitPlugin::goalCallback, this);
  feedbackSubscriber_ = getNodeHandle().subscribe<
      free_gait_msgs::ExecuteStepsActionFeedback>(
      getNodeHandle().param<std::string>("/free_gait/action_server", "") +
          "/feedback", 10, &FreeGaitPlugin::feedbackCallback, this);
  resultSubscriber_ = getNodeHandle().subscribe<
      free_gait_msgs::ExecuteStepsActionResult>(
      getNodeHandle().param<std::string>("/free_gait/action_server", "") +
          "/result", 10, &FreeGaitPlugin::resultCallback, this);

  // Initialize service clients.
  pauseClient_ = getNodeHandle().serviceClient<std_srvs::SetBool>(
      getNodeHandle().param<std::string>(
          "/free_gait/pause_execution_service", ""), false);

  stopClient_ = getNodeHandle().serviceClient<std_srvs::Trigger>(
      getNodeHandle().param<std::string>(
          "/free_gait/stop_execution_service", ""), false);

  // Initialize progress bar.
  ui_.progressBarAll->setMinimum(0);
  ui_.progressBarAll->setMaximum(1);
  ui_.progressBarAll->setValue(1);
  ui_.progressBarAll->setFormat("");

  ui_.progressBarStep->setMinimum(0);
  ui_.progressBarStep->setMaximum(1);
  ui_.progressBarStep->setValue(1);
  ui_.progressBarStep->setFormat("");

  // Set icon.
  ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/done.svg"));

  // Initialize buttons.
  ui_.pushButtonPlay->setEnabled(false);
  ui_.pushButtonPause->setEnabled(false);
  ui_.pushButtonStop->setEnabled(false);

  // Initialize navigation buttons.
  updateNavigationButtonStates();
  ui_.widgetScrollArea->installEventFilter(this);
  ui_.pushButtonGoTop->installEventFilter(this);
  ui_.pushButtonGoUp->installEventFilter(this);
  ui_.pushButtonGoDown->installEventFilter(this);
  ui_.pushButtonGoBottom->installEventFilter(this);

  // Connect signals and slots.
  connect(this,
          SIGNAL(updateGoalSignal(free_gait_msgs::ExecuteStepsActionGoal)),
          this,
          SLOT(updateGoal(free_gait_msgs::ExecuteStepsActionGoal)));
  connect(this,
          SIGNAL(updateFeedbackSignal(
              free_gait_msgs::ExecuteStepsActionFeedback)),
          this,
          SLOT(updateFeedback(free_gait_msgs::ExecuteStepsActionFeedback)));
  connect(this,
          SIGNAL(updateResultSignal(free_gait_msgs::ExecuteStepsActionResult)),
          this,
          SLOT(updateResult(free_gait_msgs::ExecuteStepsActionResult)));

  connect(ui_.pushButtonGoTop, SIGNAL(clicked()),
          this, SLOT(onPushButtonGoTop()));
  connect(ui_.pushButtonGoUp, SIGNAL(clicked()),
          this, SLOT(onPushButtonGoUp()));
  connect(ui_.pushButtonGoDown, SIGNAL(clicked()),
          this, SLOT(onPushButtonGoDown()));
  connect(ui_.pushButtonGoBottom, SIGNAL(clicked()),
          this, SLOT(onPushButtonGoBottom()));

  connect(ui_.pushButtonPlay, SIGNAL(clicked()),
          this, SLOT(onPushButtonPlay()));
  connect(ui_.pushButtonPause, SIGNAL(clicked()),
          this, SLOT(onPushButtonPause()));
  connect(ui_.pushButtonStop, SIGNAL(clicked()),
          this, SLOT(onPushButtonStop()));

  isActionRunning_ = false;
}

void FreeGaitPlugin::shutdownPlugin() {
  goalSubscriber_.shutdown();
  feedbackSubscriber_.shutdown();
  resultSubscriber_.shutdown();
}

/*****************************************************************************/
/** Settings                                                                **/
/*****************************************************************************/

void FreeGaitPlugin::saveSettings(
    qt_gui_cpp::Settings &plugin_settings,
    qt_gui_cpp::Settings &instance_settings) const {
}

void FreeGaitPlugin::restoreSettings(
    const qt_gui_cpp::Settings &plugin_settings,
    const qt_gui_cpp::Settings &instance_settings) {
}

/*****************************************************************************/
/** Callbacks                                                               **/
/*****************************************************************************/

void FreeGaitPlugin::goalCallback(
    const free_gait_msgs::ExecuteStepsActionGoalConstPtr &goal) {
  totalSteps_ = goal->goal.steps.size();

  emit updateGoalSignal(*goal);

  isActionRunning_ = true;
}

void FreeGaitPlugin::feedbackCallback(
    const free_gait_msgs::ExecuteStepsActionFeedbackConstPtr &feedback) {
  if (!isActionRunning_) {
    return;
  }

  emit updateFeedbackSignal(*feedback);
}

void FreeGaitPlugin::resultCallback(
    const free_gait_msgs::ExecuteStepsActionResultConstPtr &result) {
  isActionRunning_ = false;

  emit updateResultSignal(*result);
}

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

void FreeGaitPlugin::updateNavigationButtonStates() {
  if (descriptions_.size() > 0) {
    ui_.labelStepNumber->setText(QString::number(descriptions_.index()));
    ui_.labelStepMax->setText(QString::number(descriptions_.size() - 1));
  } else {
    ui_.labelStepNumber->setText("...");
    ui_.labelStepMax->setText("...");
  }

  if (isOnBottom_) {
    ui_.pushButtonGoBottom->setEnabled(false);
    ui_.pushButtonGoDown->setEnabled(false);
    ui_.pushButtonGoUp->setEnabled(descriptions_.size() > 1);
    ui_.pushButtonGoTop->setEnabled(descriptions_.size() > 1);
    return;
  }

  ui_.pushButtonGoBottom->setEnabled(true);
  ui_.pushButtonGoDown->setEnabled(
      descriptions_.size() > 1 && descriptions_.index() !=
                                      descriptions_.size()-1);
  ui_.pushButtonGoUp->setEnabled(
      descriptions_.size() > 1 && descriptions_.index() != 0);
  ui_.pushButtonGoTop->setEnabled(
      descriptions_.size() > 1 && descriptions_.index() != 0);
}

/*****************************************************************************/
/** Events                                                                  **/
/*****************************************************************************/

bool FreeGaitPlugin::eventFilter(QObject *object, QEvent *event) {
  if (event->type() == QEvent::Wheel) {
    QWheelEvent *wheelEvent = static_cast<QWheelEvent *>(event);
    int numDegrees = wheelEvent->delta() / 8;
    int numSteps = numDegrees / 15;

    descriptions_.moveIndex(-numSteps);
    ui_.plainTextEditDescription->setPlainText(descriptions_.currentQString());
    isOnBottom_ = false;
    updateNavigationButtonStates();

    wheelEvent->accept();
  }
  return QObject::eventFilter(object, event);
}

/*****************************************************************************/
/** Slots                                                                   **/
/*****************************************************************************/

void FreeGaitPlugin::updateGoal(free_gait_msgs::ExecuteStepsActionGoal goal) {
  // init progress bar
  ui_.progressBarAll->setMinimum(0);
  ui_.progressBarAll->setMaximum(
      (int)(progressBarMultiplicator_ * totalSteps_));
  ui_.progressBarAll->setValue(0);
  std::stringstream progressBarTextFreeGait;
  progressBarTextFreeGait << 0 << "/" << totalSteps_ << " steps";
  ui_.progressBarAll->setFormat(QString::fromStdString(
      progressBarTextFreeGait.str()));

  ui_.progressBarStep->setMinimum(0);
  ui_.progressBarStep->setMaximum(1);
  ui_.progressBarStep->setValue(0);
  ui_.progressBarStep->setFormat("");

  // reset text
  updateNavigationButtonStates();

  ui_.plainTextEditDescription->setPlainText(" ");

  // pause/play button
  ui_.pushButtonStop->setEnabled(true);
  ui_.pushButtonPause->setEnabled(true);
  ui_.pushButtonPlay->setEnabled(false);

  // update status
  ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/play.svg"));
}

void FreeGaitPlugin::updateFeedback(
    free_gait_msgs::ExecuteStepsActionFeedback feedback) {
  // update progress bar
  double freeGaitProgress = ((double)totalSteps_ -
      (double)feedback.feedback.queue_size) + feedback.feedback.phase;
  ui_.progressBarAll->setValue((int)(
      progressBarMultiplicator_ * freeGaitProgress));
  std::stringstream progressBarTextFreeGait;
  progressBarTextFreeGait << (totalSteps_ - feedback.feedback.queue_size)
                          << "/" << totalSteps_ << " steps";
  ui_.progressBarAll->setFormat(QString::fromStdString(
      progressBarTextFreeGait.str()));

  double stepProgress = feedback.feedback.phase *
      feedback.feedback.duration.toSec();
  double stepMaximum = feedback.feedback.duration.toSec();
  ui_.progressBarStep->setMinimum(0);
  ui_.progressBarStep->setMaximum((int)(
      progressBarMultiplicator_ * stepMaximum));
  ui_.progressBarStep->setValue((int)(
      progressBarMultiplicator_ * stepProgress));
  std::stringstream progressBarText;
  progressBarText << std::fixed << std::setprecision(2);
  progressBarText << stepProgress << "/" << stepMaximum << " s";
  ui_.progressBarStep->setFormat(QString::fromStdString(progressBarText.str()));

  // update text
  if (!feedback.feedback.description.empty()) {
    description_t description;
    description.message = QString::fromStdString(
        feedback.feedback.description);
    boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    std::string timestamp = boost::posix_time::to_simple_string(now);
    description.timestamp = QString::fromStdString(timestamp);
    descriptions_.push_back(description);

    if (isOnBottom_) {
      descriptions_.moveIndexBack();
      ui_.plainTextEditDescription->setPlainText(descriptions_.backQString());
    } else {
      ui_.plainTextEditDescription->setPlainText(descriptions_.currentQString());
    }
  }
  updateNavigationButtonStates();

  // update legs
  QString activeBranches = "";
  for (auto active_branch : feedback.feedback.active_branches) {
    activeBranches += QString::fromStdString(active_branch) + "  ";
  }
  ui_.labelActiveBranches->setText(activeBranches);

  // update status
  switch (feedback.feedback.status) {
    case free_gait_msgs::ExecuteStepsFeedback::PROGRESS_PAUSED:
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/pause.svg"));
      break;
    case free_gait_msgs::ExecuteStepsFeedback::PROGRESS_EXECUTING:
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/play.svg"));
      break;
    case free_gait_msgs::ExecuteStepsFeedback::PROGRESS_UNKNOWN:
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/unknown.svg"));
      break;
    default:
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/warning.svg"));
      break;
  }
}

void FreeGaitPlugin::updateResult(
    free_gait_msgs::ExecuteStepsActionResult result) {
  // reset progress bar
  ui_.progressBarAll->setMinimum(0);
  ui_.progressBarAll->setMaximum(1);
  ui_.progressBarAll->setValue(1);
  ui_.progressBarAll->setFormat("");

  ui_.progressBarStep->setMinimum(0);
  ui_.progressBarStep->setMaximum(1);
  ui_.progressBarStep->setValue(1);
  ui_.progressBarStep->setFormat("");

  // reset legs
  ui_.labelActiveBranches->setText("...");

  // reset status
  switch (result.result.status) {
    case free_gait_msgs::ExecuteStepsResult::RESULT_REACHED:
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/done.svg"));
      break;
    case free_gait_msgs::ExecuteStepsResult::RESULT_FAILED:
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/failed.svg"));
      break;
    case free_gait_msgs::ExecuteStepsResult::RESULT_UNKNOWN:
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/unknown.svg"));
      break;
    default:
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/warning.svg"));
      break;
  }

  // pause/play button
  ui_.pushButtonStop->setEnabled(false);
  ui_.pushButtonPause->setEnabled(false);
  ui_.pushButtonPlay->setEnabled(false);
}

void FreeGaitPlugin::onPushButtonGoTop() {
  descriptions_.moveIndexFront();
  ui_.plainTextEditDescription->setPlainText(descriptions_.currentQString());
  isOnBottom_ = false;

  updateNavigationButtonStates();
}

void FreeGaitPlugin::onPushButtonGoUp() {
  descriptions_.moveIndex(-1);
  ui_.plainTextEditDescription->setPlainText(descriptions_.currentQString());
  isOnBottom_ = false;

  updateNavigationButtonStates();
}

void FreeGaitPlugin::onPushButtonGoDown() {
  descriptions_.moveIndex(1);
  ui_.plainTextEditDescription->setPlainText(descriptions_.currentQString());
  isOnBottom_ = false;

  updateNavigationButtonStates();
}

void FreeGaitPlugin::onPushButtonGoBottom() {
  descriptions_.moveIndexBack();
  ui_.plainTextEditDescription->setPlainText(descriptions_.backQString());
  isOnBottom_ = true;

  updateNavigationButtonStates();
}

void FreeGaitPlugin::onPushButtonPlay() {
  ui_.pushButtonPause->setEnabled(false);
  ui_.pushButtonPlay->setEnabled(false);

  std_srvs::SetBoolRequest request;
  request.data = false;

  WorkerThreadPausePlay *workerThreadPausePlay = new WorkerThreadPausePlay;
  connect(workerThreadPausePlay,
          SIGNAL(result(bool, std_srvs::SetBoolResponse)),
          this,
          SLOT(onPushButtonPlayResult(bool, std_srvs::SetBoolResponse)));
  connect(workerThreadPausePlay, SIGNAL(finished()),
          workerThreadPausePlay, SLOT(deleteLater()));
  workerThreadPausePlay->setClient(pauseClient_);
  workerThreadPausePlay->setRequest(request);
  workerThreadPausePlay->start();
}

void FreeGaitPlugin::onPushButtonPause() {
  ui_.pushButtonPause->setEnabled(false);
  ui_.pushButtonPlay->setEnabled(false);

  std_srvs::SetBoolRequest request;
  request.data = true;

  WorkerThreadPausePlay *workerThreadPausePlay = new WorkerThreadPausePlay;
  connect(workerThreadPausePlay,
          SIGNAL(result(bool, std_srvs::SetBoolResponse)),
          this,
          SLOT(onPushButtonPauseResult(bool, std_srvs::SetBoolResponse)));
  connect(workerThreadPausePlay, SIGNAL(finished()),
          workerThreadPausePlay, SLOT(deleteLater()));
  workerThreadPausePlay->setClient(pauseClient_);
  workerThreadPausePlay->setRequest(request);
  workerThreadPausePlay->start();
}

void FreeGaitPlugin::onPushButtonStop() {
  ui_.pushButtonStop->setEnabled(false);
  ui_.pushButtonPause->setEnabled(false);
  ui_.pushButtonPlay->setEnabled(false);

  WorkerThreadStop *workerThreadStop = new WorkerThreadStop;
  connect(workerThreadStop,
          SIGNAL(result(bool, std_srvs::TriggerResponse)),
          this,
          SLOT(onPushButtonStopResult(bool, std_srvs::TriggerResponse)));
  connect(workerThreadStop, SIGNAL(finished()),
          workerThreadStop, SLOT(deleteLater()));
  workerThreadStop->setClient(stopClient_);
  workerThreadStop->start();
}

void FreeGaitPlugin::onPushButtonPlayResult(
    bool isOk, std_srvs::SetBoolResponse response) {
  if (isOk && response.success) {
    ui_.pushButtonPause->setEnabled(true);
    ui_.pushButtonPlay->setEnabled(false);
  } else {
    ui_.pushButtonPause->setEnabled(false);
    ui_.pushButtonPlay->setEnabled(true);
  }
}

void FreeGaitPlugin::onPushButtonPauseResult(
    bool isOk, std_srvs::SetBoolResponse response) {
  if (isOk && response.success) {
    ui_.pushButtonPause->setEnabled(false);
    ui_.pushButtonPlay->setEnabled(true);
  } else {
    ui_.pushButtonPause->setEnabled(true);
    ui_.pushButtonPlay->setEnabled(false);
  }
}

void FreeGaitPlugin::onPushButtonStopResult(
    bool isOk, std_srvs::TriggerResponse response) {
  if (isOk && response.success) {
    ui_.pushButtonStop->setEnabled(false);
    ui_.pushButtonPause->setEnabled(false);
    ui_.pushButtonPlay->setEnabled(false);
  } else {
    ui_.pushButtonStop->setEnabled(true);
    ui_.pushButtonPause->setEnabled(true);
    ui_.pushButtonPlay->setEnabled(false);
  }
}

} // namespace

PLUGINLIB_EXPORT_CLASS(rqt_free_gait::FreeGaitPlugin, rqt_gui_cpp::Plugin)
