#pragma once

#include <QString>
#include <QPushButton>
#include "rviz/properties/property.h"
#include <string>

namespace rviz
{

class ButtonProperty: public Property
{
Q_OBJECT
public:
ButtonProperty(const QString& name = QString(), const QString& default_value = QString(),
                 const QString& description = QString(), Property* parent = 0,
                 const char* changed_slot = 0, QObject* receiver = 0);

  void setLabel(const std::string& label);
  QVariant getViewData( int column, int role ) const;
  Qt::ItemFlags getViewFlags( int column ) const;
  virtual QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option);

public Q_SLOTS:
  void buttonReleased();

private:
  QString label_;
};

} // end namespace rviz

