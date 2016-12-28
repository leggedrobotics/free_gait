/*
 * line_edit_with_slider.cpp
 *
 *  Created on: Dec 12, 2016
 *  Author: Péter Fankhauser
 *  Institute: ETH Zurich, Robotic Systems Lab
 */

#include "rviz/properties/line_edit_with_slider.h"

#include <QSlider>

namespace rviz
{

LineEditWithSlider::LineEditWithSlider( QWidget* parent )
  : QLineEdit( parent )
{
  setFrame(false);
  slider_ = new QSlider(Qt::Orientation::Horizontal, this);
}

void LineEditWithSlider::resizeEvent( QResizeEvent* event )
{
  int padding = 0;
  int textMinimalWidth = 45;
  int sliderWidth = width() - textMinimalWidth - 2;
  setTextMargins(padding, 0, sliderWidth, 0);
  QLineEdit::resizeEvent(event);
  slider_->setGeometry(textMinimalWidth, padding, sliderWidth, height());
}

} // end namespace rviz
