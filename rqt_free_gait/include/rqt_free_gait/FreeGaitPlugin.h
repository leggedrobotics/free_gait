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
#include <ui_FreeGaitPlugin.h>

#include <atomic>
#include <mutex>

#include <ros/ros.h>
#include <ros/package.h>

#include <free_gait_msgs/ExecuteStepsActionGoal.h>
#include <free_gait_msgs/ExecuteStepsActionFeedback.h>
#include <free_gait_msgs/ExecuteStepsActionResult.h>

#include <QWidget>
#include <QObject>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QEvent>
#include <QWheelEvent>

namespace rqt_free_gait {

class FreeGaitPlugin : public rqt_gui_cpp::Plugin {
  Q_OBJECT
public:

  /***************************************************************************/
  /** Constructor/Destructor                                                **/
  /***************************************************************************/

  FreeGaitPlugin();

  /***************************************************************************/
  /** Initialization/Shutdown                                               **/
  /***************************************************************************/

  virtual void initPlugin(qt_gui_cpp::PluginContext &context);

  virtual void shutdownPlugin();

  /***************************************************************************/
  /** Settings                                                              **/
  /***************************************************************************/

  virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                            qt_gui_cpp::Settings &instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                               const qt_gui_cpp::Settings &instance_settings);

protected:

  /***************************************************************************/
  /** Constants                                                             **/
  /***************************************************************************/

  const std::string TAG = "FreeGaitPlugin";

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  Ui::FreeGaitPluginWidget ui_;
  QWidget *widget_;

  ros::NodeHandle nh_;

  ros::Subscriber goalSubscriber_;
  ros::Subscriber feedbackSubscriber_;
  ros::Subscriber resultSubscriber_;

  const double progressBarMultiplicator_ = 1000.0;
  int totalSteps_ = 0;

  std::atomic<bool> isActionRunning_;

  std::vector<QString> descriptions_;
  int descriptionIndex_ = 0;
  bool isOnBottom_ = true;

  /***************************************************************************/
  /** Callbacks                                                             **/
  /***************************************************************************/

  void goalCallback(const free_gait_msgs::ExecuteStepsActionGoalConstPtr &goal);

  void feedbackCallback(
      const free_gait_msgs::ExecuteStepsActionFeedbackConstPtr &feedback);

  void resultCallback(
      const free_gait_msgs::ExecuteStepsActionResultConstPtr &result);

  /***************************************************************************/
  /** Methods                                                               **/
  /***************************************************************************/

  void updateNavigationButtonStates();

  /***************************************************************************/
  /** Events                                                                **/
  /***************************************************************************/

  bool eventFilter(QObject *object, QEvent *event);

protected slots:

  /***************************************************************************/
  /** Slots                                                                 **/
  /***************************************************************************/

  void updateGoal(free_gait_msgs::ExecuteStepsActionGoal goal);

  void updateFeedback(free_gait_msgs::ExecuteStepsActionFeedback feedback);

  void updateResult(free_gait_msgs::ExecuteStepsActionResult result);

  void onPushButtonGoTop();

  void onPushButtonGoUp();

  void onPushButtonGoDown();

  void onPushButtonGoBottom();

signals:

  /***************************************************************************/
  /** Signals                                                               **/
  /***************************************************************************/

  void updateGoalSignal(free_gait_msgs::ExecuteStepsActionGoal goal);

  void updateFeedbackSignal(
      free_gait_msgs::ExecuteStepsActionFeedback feedback);

  void updateResultSignal(free_gait_msgs::ExecuteStepsActionResult result);

};

} // namespace
