#include "rviz/properties/button_property.h"

namespace rviz
{

ButtonProperty::ButtonProperty(const QString& name, const QString& default_value,
                               const QString& description, Property* parent,
                               const char *changed_slot, QObject* receiver)
    : Property(name, default_value, description, parent, changed_slot, receiver),
      label_(default_value)
{
  setShouldBeSaved(false);
}

void ButtonProperty::setLabel(const std::string& label)
{
  label_ = QString::fromStdString(label);
}

QVariant ButtonProperty::getViewData( int column, int role ) const
{
  if (column == 1) {
    switch (role) {
      case Qt::DisplayRole:
        return label_;
      case Qt::EditRole:
      case Qt::CheckStateRole:
        return QVariant();
      default:
        return Property::getViewData(column, role);
    }
  }
  return Property::getViewData(column, role);
}

Qt::ItemFlags ButtonProperty::getViewFlags( int column ) const
{
  Qt::ItemFlags enabled_flag = ( getParent() && getParent()->getDisableChildren() ) ? Qt::NoItemFlags : Qt::ItemIsEnabled;
  if (column == 0) return Property::getViewFlags(column);
  return Qt::ItemIsEditable | enabled_flag | Qt::ItemIsSelectable;
}

QWidget* ButtonProperty::createEditor(QWidget* parent, const QStyleOptionViewItem& option)
{
  QPushButton* button = new QPushButton(label_, parent);
  connect(button, SIGNAL(released()), this, SLOT(buttonReleased()));
  return button;
}

void ButtonProperty::buttonReleased()
{
  Q_EMIT changed();
}

} // end namespace rviz
