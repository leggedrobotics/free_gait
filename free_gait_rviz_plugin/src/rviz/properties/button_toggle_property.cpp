#include "rviz/properties/button_toggle_property.h"

namespace rviz
{

ButtonToggleProperty::ButtonToggleProperty(const QString& name, bool default_value,
                                           const QString& description, Property* parent,
                                           const char *changed_slot, QObject* receiver)
    : ButtonProperty(name, "", description, parent, changed_slot, receiver),
      labelForTrue_("True"),
      labelForFalse_("False"),
      button_(NULL)
{
}

void ButtonToggleProperty::setLabels(const std::string& labelForTrue, const std::string& labelForFalse)
{
  labelForTrue_ = QString::fromStdString(labelForTrue);
  labelForFalse_ = QString::fromStdString(labelForFalse);
}

const QString& ButtonToggleProperty::getLabel()
{
  return (getBool() ? labelForTrue_ : labelForFalse_);
}

bool ButtonToggleProperty::getBool() const
{
  return getValue().toBool();
}

QWidget* ButtonToggleProperty::createEditor(QWidget* parent, const QStyleOptionViewItem& option)
{
  button_ = new QPushButton(getLabel(), parent);
  button_->setCheckable(true);
  button_->setChecked(true);
  connect(button_, SIGNAL(toggled(bool)), this, SLOT(buttonToggled(bool)));
  return button_;
}

void ButtonToggleProperty::buttonToggled(bool state)
{
  button_->setText(getLabel());
  value_.setValue(state);
  Q_EMIT changed();
}

} // end namespace rviz
