/*
 * float_slider_property.h
 *
 *  Created on: Dec 12, 2016
 *  Author: Péter Fankhauser
 *  Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include <rviz/properties/float_property.h>
#include <QString>
#include <string>

namespace rviz {

class FloatSliderProperty : public FloatProperty
{
 Q_OBJECT
 public:
  FloatSliderProperty(const QString& name = QString(), float default_value = 0.0,
                      const QString& description = QString(), Property* parent = 0,
                      const char* changed_slot = 0, QObject* receiver = 0);

  virtual QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option);

  virtual bool setValuePassive(const QVariant& value);

public Q_SLOTS:
  virtual void valueChanged(int value);

 private:
  int minIntValue_, maxIntValue_;
};

}  // end namespace rviz

