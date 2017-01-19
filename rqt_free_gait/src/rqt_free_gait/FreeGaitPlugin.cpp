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
    : rqt_gui_cpp::Plugin(), widget_(0) {

  setObjectName("FreeGaitPlugin");

  qRegisterMetaType<free_gait_msgs::ExecuteStepsActionGoal>
      ("free_gait_msgs::ExecuteStepsActionGoal");
  qRegisterMetaType<free_gait_msgs::ExecuteStepsActionFeedback>
      ("free_gait_msgs::ExecuteStepsActionFeedback");
  qRegisterMetaType<free_gait_msgs::ExecuteStepsActionResult>
      ("free_gait_msgs::ExecuteStepsActionResult");
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

  // Initialize name.
  ui_.labelName->setText("<none>");

  // Initialize progress bar.
  ui_.progressBarAll->setMinimum(0);
  ui_.progressBarAll->setMaximum(1);
  ui_.progressBarAll->setValue(1);
  ui_.progressBarAll->setFormat("");

  ui_.progressBarStep->setMinimum(0);
  ui_.progressBarStep->setMaximum(1);
  ui_.progressBarStep->setValue(1);
  ui_.progressBarStep->setFormat("");

  // Set icons.
  std::string iconPath =
      ros::package::getPath("rqt_free_gait") + "/resource/icons/16x16/";
  std::string s;
  s = iconPath + "done.svg";
  pixmapDone_ = QPixmap(s.c_str());
  s = iconPath + "failed.svg";
  pixmapFailed_ = QPixmap(s.c_str());
  s = iconPath + "pause.svg";
  pixmapPause_ = QPixmap(s.c_str());
  s = iconPath + "play.svg";
  pixmapPlay_ = QPixmap(s.c_str());
  s = iconPath + "stop.svg";
  pixmapStop_ = QPixmap(s.c_str());
  s = iconPath + "unknown.svg";
  pixmapUnknown_ = QPixmap(s.c_str());
  s = iconPath + "warning.svg";
  pixmapWarning_ = QPixmap(s.c_str());
  ui_.labelStatus->setPixmap(pixmapDone_);

  // Initialize foot labels.
  ui_.labelLF->setStyleSheet("QLabel {color: black;}");
  ui_.labelRF->setStyleSheet("QLabel {color: black;}");
  ui_.labelLH->setStyleSheet("QLabel {color: black;}");
  ui_.labelRH->setStyleSheet("QLabel {color: black;}");

  // Initialize buttons.
  ui_.pushButtonPlay->setEnabled(false);
  ui_.pushButtonPause->setEnabled(false);
  ui_.pushButtonStop->setEnabled(false);

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

  // update status
  ui_.labelStatus->setPixmap(pixmapPlay_);
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
  mutexExtendText_.lock();
  ui_.plainTextEditDescription->setPlainText(
      QString::fromStdString(feedback.feedback.description));
  mutexExtendText_.unlock();

  // update legs
  if (std::find(feedback.feedback.swing_leg_names.begin(),
                feedback.feedback.swing_leg_names.end(), "LF_LEG") !=
      feedback.feedback.swing_leg_names.end()) {
    ui_.labelLF->setStyleSheet("QLabel {color: green;}");
  } else {
    ui_.labelLF->setStyleSheet("QLabel {color: black;}");
  }
  if (std::find(feedback.feedback.swing_leg_names.begin(),
                feedback.feedback.swing_leg_names.end(), "RF_LEG") !=
      feedback.feedback.swing_leg_names.end()) {
    ui_.labelRF->setStyleSheet("QLabel {color: green;}");
  } else {
    ui_.labelRF->setStyleSheet("QLabel {color: black;}");
  }
  if (std::find(feedback.feedback.swing_leg_names.begin(),
                feedback.feedback.swing_leg_names.end(), "LH_LEG") !=
      feedback.feedback.swing_leg_names.end()) {
    ui_.labelLH->setStyleSheet("QLabel {color: green;}");
  } else {
    ui_.labelLH->setStyleSheet("QLabel {color: black;}");
  }
  if (std::find(feedback.feedback.swing_leg_names.begin(),
                feedback.feedback.swing_leg_names.end(), "RH_LEG") !=
      feedback.feedback.swing_leg_names.end()) {
    ui_.labelRH->setStyleSheet("QLabel {color: green;}");
  } else {
    ui_.labelRH->setStyleSheet("QLabel {color: black;}");
  }

  // update status
  switch (feedback.feedback.status) {
    case free_gait_msgs::ExecuteStepsFeedback::PROGRESS_PAUSED:
      ui_.labelStatus->setPixmap(pixmapPause_);
      break;
    case free_gait_msgs::ExecuteStepsFeedback::PROGRESS_EXECUTING:
      ui_.labelStatus->setPixmap(pixmapPlay_);
      break;
    case free_gait_msgs::ExecuteStepsFeedback::PROGRESS_UNKNOWN:
      ui_.labelStatus->setPixmap(pixmapUnknown_);
      break;
    default:
      ui_.labelStatus->setPixmap(pixmapWarning_);
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

  // reset text
  ui_.labelName->setText("<none>");
  mutexExtendText_.lock();
  ui_.plainTextEditDescription->setPlainText(" ");
  mutexExtendText_.unlock();

  // reset legs
  ui_.labelLF->setStyleSheet("QLabel {color: black;}");
  ui_.labelRF->setStyleSheet("QLabel {color: black;}");
  ui_.labelLH->setStyleSheet("QLabel {color: black;}");
  ui_.labelRH->setStyleSheet("QLabel {color: black;}");

  // reset status
  switch (result.result.status) {
    case free_gait_msgs::ExecuteStepsResult::RESULT_REACHED:
      ui_.labelStatus->setPixmap(pixmapDone_);
      break;
    case free_gait_msgs::ExecuteStepsResult::RESULT_FAILED:
      ui_.labelStatus->setPixmap(pixmapFailed_);
      break;
    case free_gait_msgs::ExecuteStepsResult::RESULT_UNKNOWN:
      ui_.labelStatus->setPixmap(pixmapUnknown_);
      break;
    default:
      ui_.labelStatus->setPixmap(pixmapWarning_);
      break;
  }
}

} // namespace

PLUGINLIB_EXPORT_CLASS(rqt_free_gait::FreeGaitPlugin, rqt_gui_cpp::Plugin
)

