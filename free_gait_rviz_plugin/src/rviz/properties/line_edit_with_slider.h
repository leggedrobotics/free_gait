/*
 * line_edit_with_slider.h
 *
 *  Created on: Dec 12, 2016
 *  Author: Péter Fankhauser
 *  Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include <QLineEdit>

class QSlider;

namespace rviz {

/**
 * A QLineEdit for a numeric value with a slider to the right.
 */
class LineEditWithSlider : public QLineEdit
{
 Q_OBJECT
 public:
  LineEditWithSlider(QWidget* parent = 0);

  /** Returns the child slider. Use this to connect() something to a
   * changed value. */
  QSlider* slider() { return slider_; }

 protected:
  virtual void resizeEvent(QResizeEvent* event);

 private:
  QSlider* slider_;
};

}  // end namespace rviz
