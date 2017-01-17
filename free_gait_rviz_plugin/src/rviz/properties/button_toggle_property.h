#pragma once

#include "rviz/properties/button_property.h"

namespace rviz
{

class ButtonToggleProperty: public ButtonProperty
{
Q_OBJECT
public:
  ButtonToggleProperty(const QString& name = QString(), bool default_value = false,
                       const QString& description = QString(), Property* parent = 0,
                       const char* changed_slot = 0, QObject* receiver = 0);

  void setLabels(const std::string& labelForTrue, const std::string& labelForFalse);
  const QString& getLabel();
  bool getBool() const;

public Q_SLOTS:
  void buttonToggled(bool state);

protected:
  virtual QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option);

  QString labelForTrue_;
  QString labelForFalse_;
  QPushButton* button_;
};

} // end namespace rviz

