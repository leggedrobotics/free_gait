#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <ui_FreeGaitPlugin.h>

#include <atomic>

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

namespace rqt_free_gait {

class FreeGaitPlugin
    : public rqt_gui_cpp::Plugin {

Q_OBJECT

public:
  /**
   * @brief FreeGaitPlugin
   */
  FreeGaitPlugin();

  /**
   * @brief initPlugin
   * @param context
   */
  virtual void initPlugin(qt_gui_cpp::PluginContext &context);

  /**
   * @brief shutdownPlugin
   */
  virtual void shutdownPlugin();

  /**
   * @brief saveSettings
   * @param plugin_settings
   * @param instance_settings
   */
  virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                            qt_gui_cpp::Settings &instance_settings) const;

  /**
   * @brief restoreSettings
   * @param plugin_settings
   * @param instance_settings
   */
  virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                               const qt_gui_cpp::Settings &instance_settings);

protected slots:

  /**
   * @brief updateGoal
   * @param goal
   */
  void updateGoal(free_gait_msgs::ExecuteStepsActionGoal goal);

  /**
   * @brief updateFeedback
   * @param feedback
   */
  void updateFeedback(free_gait_msgs::ExecuteStepsActionFeedback feedback);

  /**
   * @brief updateResult
   * @param result
   */
  void updateResult(free_gait_msgs::ExecuteStepsActionResult result);

  /**
   * @brief on_pushButton_preempt_clicked
   */
  void on_pushButton_preempt_clicked();

signals:

  /**
   * @brief updateGoalSignal
   * @param goal
   */
  void updateGoalSignal(free_gait_msgs::ExecuteStepsActionGoal goal);

  /**
   * @brief updateFeedbackSignal
   * @param feedback
   */
  void updateFeedbackSignal(free_gait_msgs::ExecuteStepsActionFeedback feedback);

  /**
   * @brief updateResultSignal
   * @param result
   */
  void updateResultSignal(free_gait_msgs::ExecuteStepsActionResult result);

protected:
  Ui::FreeGaitPluginWidget ui_;
  QWidget *widget_;
  ros::NodeHandle nh_;

  ros::Subscriber subGoal_;
  ros::Subscriber subFeedback_;
  ros::Subscriber subResult_;

  const double progressBarMultiplicator_ = 1000.0;
  int totalSteps_ = 0;

  std::atomic<bool> isActionRunning_;

  QPixmap pixmapDone_;
  QPixmap pixmapFailed_;
  QPixmap pixmapPause_;
  QPixmap pixmapPlay_;
  QPixmap pixmapStop_;
  QPixmap pixmapUnknown_;
  QPixmap pixmapWarning_;

  /**
   * @brief goalCallback
   * @param goal
   */
  void goalCallback(const free_gait_msgs::ExecuteStepsActionGoalConstPtr &goal);

  /**
   * @brief feedbackCallback
   * @param feedback
   */
  void feedbackCallback(const free_gait_msgs::ExecuteStepsActionFeedbackConstPtr &feedback);

  /**
   * @brief resultCallback
   * @param result
   */
  void resultCallback(const free_gait_msgs::ExecuteStepsActionResultConstPtr &result);
};

} // namespace
