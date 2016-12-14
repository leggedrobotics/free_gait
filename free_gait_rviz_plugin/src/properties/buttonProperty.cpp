#include "free_gait_rviz_plugin/properties/buttonProperty.hpp"

//#include <QColor>

namespace free_gait_rviz_plugin {

ButtonProperty::ButtonProperty(const QString& name, bool default_value, const QString& description,
                               Property* parent, const char *changed_slot, QObject* receiver)
    : Property(name, default_value, description, parent, changed_slot, receiver),
      disable_children_if_false_(false)
{
}

ButtonProperty::~ButtonProperty()
{
}

bool ButtonProperty::getBool() const
{
  return getValue().toBool();
}

void ButtonProperty::setDisableChildrenIfFalse(bool disable)
{
  disable_children_if_false_ = disable;
}

bool ButtonProperty::getDisableChildrenIfFalse()
{
  return disable_children_if_false_;
}

bool ButtonProperty::getDisableChildren()
{
  if (disable_children_if_false_) {
    return !getBool() || Property::getDisableChildren();
  }
  return Property::getDisableChildren();
}

}  // end namespace
