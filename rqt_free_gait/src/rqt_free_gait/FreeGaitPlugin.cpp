#include "rqt_free_gait/FreeGaitPlugin.h"

#include <pluginlib/class_list_macros.h>

namespace rqt_free_gait {

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

void FreeGaitPlugin::initPlugin(qt_gui_cpp::PluginContext &context) {
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // subscriber
  subGoal_ = getNodeHandle().subscribe<free_gait_msgs::ExecuteStepsActionGoal>
      ("/loco_free_gait/execute_steps/goal", 10, &FreeGaitPlugin::goalCallback, this);
  subFeedback_ = getNodeHandle().subscribe<free_gait_msgs::ExecuteStepsActionFeedback>
      ("/loco_free_gait/execute_steps/feedback", 10, &FreeGaitPlugin::feedbackCallback, this);
  subResult_ = getNodeHandle().subscribe<free_gait_msgs::ExecuteStepsActionResult>
      ("/loco_free_gait/execute_steps/result", 10, &FreeGaitPlugin::resultCallback, this);

  // init gui
  ui_.label_name->setText("<none>");

  ui_.progressBar_free_gait->setMinimum(0);
  ui_.progressBar_free_gait->setMaximum(0);
  ui_.progressBar_free_gait->setValue(0);
//  ui_.progressBar_free_gait->setEnabled(false);

  std::string s;
  s = ros::package::getPath("rqt_free_gait") + "/resource/16x16/" + "done.svg";
  pixmapDone_ = QPixmap(s.c_str());
  s = ros::package::getPath("rqt_free_gait") + "/resource/16x16/" + "failed.svg";
  pixmapFailed_ = QPixmap(s.c_str());
  s = ros::package::getPath("rqt_free_gait") + "/resource/16x16/" + "pause.svg";
  pixmapPause_ = QPixmap(s.c_str());
  s = ros::package::getPath("rqt_free_gait") + "/resource/16x16/" + "play.svg";
  pixmapPlay_ = QPixmap(s.c_str());
  s = ros::package::getPath("rqt_free_gait") + "/resource/16x16/" + "stop.svg";
  pixmapStop_ = QPixmap(s.c_str());
  s = ros::package::getPath("rqt_free_gait") + "/resource/16x16/" + "unknown.svg";
  pixmapUnknown_ = QPixmap(s.c_str());
  s = ros::package::getPath("rqt_free_gait") + "/resource/16x16/" + "warning.svg";
  pixmapWarning_ = QPixmap(s.c_str());
  ui_.label_status->setPixmap(pixmapDone_);

  ui_.label_description->setText("<none>");

  ui_.progressBar_step->setMinimum(0);
  ui_.progressBar_step->setMaximum(0);
  ui_.progressBar_step->setValue(0);
//  ui_.progressBar_step->setEnabled(false);

  ui_.label_LF->setStyleSheet("QLabel {color: red;}");
  ui_.label_RF->setStyleSheet("QLabel {color: red;}");
  ui_.label_LH->setStyleSheet("QLabel {color: red;}");
  ui_.label_RH->setStyleSheet("QLabel {color: red;}");

  // connct signals and slots
  connect(this, SIGNAL(updateGoalSignal(free_gait_msgs::ExecuteStepsActionGoal)),
          this, SLOT(updateGoal(free_gait_msgs::ExecuteStepsActionGoal)));
  connect(this, SIGNAL(updateFeedbackSignal(free_gait_msgs::ExecuteStepsActionFeedback)),
          this, SLOT(updateFeedback(free_gait_msgs::ExecuteStepsActionFeedback)));
  connect(this, SIGNAL(updateResultSignal(free_gait_msgs::ExecuteStepsActionResult)),
          this, SLOT(updateResult(free_gait_msgs::ExecuteStepsActionResult)));
  connect(ui_.pushButton_preempt, SIGNAL(clicked()), this, SLOT(on_pushButton_preempt_clicked()));

  isActionRunning_ = false;
}

void FreeGaitPlugin::shutdownPlugin() {
  subGoal_.shutdown();
  subFeedback_.shutdown();
  subResult_.shutdown();
}

void FreeGaitPlugin::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                  qt_gui_cpp::Settings &instance_settings) const {
}

void FreeGaitPlugin::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                     const qt_gui_cpp::Settings &instance_settings) {
}

void FreeGaitPlugin::goalCallback(const free_gait_msgs::ExecuteStepsActionGoalConstPtr &goal) {
  totalSteps_ = goal->goal.steps.size();

  emit updateGoalSignal(*goal);

  isActionRunning_ = true;
}

void FreeGaitPlugin::feedbackCallback(const free_gait_msgs::ExecuteStepsActionFeedbackConstPtr &feedback) {
  if (!isActionRunning_) {
    return;
  }

  emit updateFeedbackSignal(*feedback);
}

void FreeGaitPlugin::resultCallback(const free_gait_msgs::ExecuteStepsActionResultConstPtr &result) {
  isActionRunning_ = false;

  emit updateResultSignal(*result);
}

void FreeGaitPlugin::updateGoal(free_gait_msgs::ExecuteStepsActionGoal goal) {
  // init progress bar
  ui_.progressBar_free_gait->setMinimum(0);
  ui_.progressBar_free_gait->setMaximum((int)(progressBarMultiplicator_ * totalSteps_));
  ui_.progressBar_free_gait->setValue(0);
  std::stringstream progressBarTextFreeGait;
  progressBarTextFreeGait << 0 << "/" << totalSteps_ << " steps";
  ui_.progressBar_free_gait->setFormat(QString::fromStdString(progressBarTextFreeGait.str()));

  // update status
  ui_.label_status->setPixmap(pixmapPlay_);
}

void FreeGaitPlugin::updateFeedback(free_gait_msgs::ExecuteStepsActionFeedback feedback) {
  // update progress bar
  double freeGaitProgress = ((double)totalSteps_ - (double)feedback.feedback.queue_size) + feedback.feedback.phase;
  ui_.progressBar_free_gait->setValue((int)(progressBarMultiplicator_ * freeGaitProgress));
  std::stringstream progressBarTextFreeGait;
  progressBarTextFreeGait << (totalSteps_ - feedback.feedback.queue_size) << "/" << totalSteps_ << " steps";
  ui_.progressBar_free_gait->setFormat(QString::fromStdString(progressBarTextFreeGait.str()));

  double stepProgress = feedback.feedback.phase * feedback.feedback.duration.toSec();
  double stepMaximum = feedback.feedback.duration.toSec();
  ui_.progressBar_step->setMinimum(0);
  ui_.progressBar_step->setMaximum((int)(progressBarMultiplicator_ * stepMaximum));
  ui_.progressBar_step->setValue((int)(progressBarMultiplicator_ * stepProgress));
  std::stringstream progressBarText;
  progressBarText << std::fixed << std::setprecision(2);
  progressBarText << stepProgress << "/" << stepMaximum << " s";
  ui_.progressBar_step->setFormat(QString::fromStdString(progressBarText.str()));

  // update text
  ui_.label_description->setText(QString::fromStdString(feedback.feedback.description));

  // update legs
  if (std::find(feedback.feedback.swing_leg_names.begin(),
                feedback.feedback.swing_leg_names.end(), "LF_LEG") != feedback.feedback.swing_leg_names.end()) {
    ui_.label_LF->setStyleSheet("QLabel {color: green;}");
  } else {
    ui_.label_LF->setStyleSheet("QLabel {color: red;}");
  }
  if (std::find(feedback.feedback.swing_leg_names.begin(),
                feedback.feedback.swing_leg_names.end(), "RF_LEG") != feedback.feedback.swing_leg_names.end()) {
    ui_.label_RF->setStyleSheet("QLabel {color: green;}");
  } else {
    ui_.label_RF->setStyleSheet("QLabel {color: red;}");
  }
  if (std::find(feedback.feedback.swing_leg_names.begin(),
                feedback.feedback.swing_leg_names.end(), "LH_LEG") != feedback.feedback.swing_leg_names.end()) {
    ui_.label_LH->setStyleSheet("QLabel {color: green;}");
  } else {
    ui_.label_LH->setStyleSheet("QLabel {color: red;}");
  }
  if (std::find(feedback.feedback.swing_leg_names.begin(),
                feedback.feedback.swing_leg_names.end(), "RH_LEG") != feedback.feedback.swing_leg_names.end()) {
    ui_.label_RH->setStyleSheet("QLabel {color: green;}");
  } else {
    ui_.label_RH->setStyleSheet("QLabel {color: red;}");
  }

  // update status
  switch (feedback.feedback.status) {
    case free_gait_msgs::ExecuteStepsFeedback::PROGRESS_PAUSED:
      ui_.label_status->setPixmap(pixmapPause_);
      break;
    case free_gait_msgs::ExecuteStepsFeedback::PROGRESS_EXECUTING:
      ui_.label_status->setPixmap(pixmapPlay_);
      break;
    case free_gait_msgs::ExecuteStepsFeedback::PROGRESS_UNKNOWN:
      ui_.label_status->setPixmap(pixmapUnknown_);
      break;
    default:
      ui_.label_status->setPixmap(pixmapWarning_);
      break;
  }
}

void FreeGaitPlugin::updateResult(free_gait_msgs::ExecuteStepsActionResult result) {
  // reset progress bar
  ui_.progressBar_free_gait->setMinimum(0);
  ui_.progressBar_free_gait->setMaximum(0);
  ui_.progressBar_free_gait->setValue(0);

  ui_.progressBar_step->setMinimum(0);
  ui_.progressBar_step->setMaximum(0);
  ui_.progressBar_step->setValue(0);

  // reset text
  ui_.label_name->setText("<none>");
  ui_.label_description->setText("<none>");

  // reset legs
  ui_.label_LF->setStyleSheet("QLabel {color: red;}");
  ui_.label_RF->setStyleSheet("QLabel {color: red;}");
  ui_.label_LH->setStyleSheet("QLabel {color: red;}");
  ui_.label_RH->setStyleSheet("QLabel {color: red;}");

  // reset status
  switch (result.result.status) {
    case free_gait_msgs::ExecuteStepsResult::RESULT_REACHED:
      ui_.label_status->setPixmap(pixmapDone_);
      break;
    case free_gait_msgs::ExecuteStepsResult::RESULT_FAILED:
      ui_.label_status->setPixmap(pixmapFailed_);
      break;
    case free_gait_msgs::ExecuteStepsResult::RESULT_UNKNOWN:
      ui_.label_status->setPixmap(pixmapUnknown_);
      break;
    default:
      ui_.label_status->setPixmap(pixmapWarning_);
      break;
  }
}

void FreeGaitPlugin::on_pushButton_preempt_clicked() {

}

} // namespace

PLUGINLIB_EXPORT_CLASS(rqt_free_gait::FreeGaitPlugin, rqt_gui_cpp::Plugin
)

