/*
 * float_slider_property.cpp
 *
 *  Created on: Dec 12, 2016
 *  Author: Péter Fankhauser
 *  Institute: ETH Zurich, Robotic Systems Lab
 */

#include "rviz/properties/float_slider_property.h"
#include "rviz/properties/line_edit_with_slider.h"

#include <QSlider>
#include <QDoubleValidator>

namespace rviz
{

FloatSliderProperty::FloatSliderProperty(const QString& name, float default_value,
                               const QString& description, Property* parent,
                               const char *changed_slot, QObject* receiver)
    : FloatProperty(name, default_value, description, parent, changed_slot, receiver),
      minIntValue_(0),
      maxIntValue_(1000)
{
  setMin(0.0);
  setMax(1.0);
}

bool FloatSliderProperty::setValuePassive(const QVariant& value)
{
  value_ = qBound(getMin(), value.toFloat(), getMax());
  return true;
}

void FloatSliderProperty::valueChanged(int value)
{
  const float floatValue = getMin() + (getMax() - getMin())
      * (value - minIntValue_) / (maxIntValue_ - minIntValue_);
  setValue(floatValue);
}

QWidget* FloatSliderProperty::createEditor(QWidget* parent, const QStyleOptionViewItem& option)
{
  LineEditWithSlider* slider = new LineEditWithSlider(parent);
  slider->slider()->setRange(minIntValue_, maxIntValue_);
  slider->setValidator(new QDoubleValidator(slider->slider()));
  const int intValue = minIntValue_ + (maxIntValue_ - minIntValue_)
      * (getValue().toFloat() - getMin()) / (getMax() - getMin());
  slider->slider()->setValue(intValue);
  connect(slider->slider(), SIGNAL(valueChanged(int)), this, SLOT(valueChanged(int)));
  return slider;
}

} // end namespace rviz
