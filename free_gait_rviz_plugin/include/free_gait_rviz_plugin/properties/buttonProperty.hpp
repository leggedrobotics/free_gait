#pragma once

#include "rviz/properties/property.h"

namespace free_gait_rviz_plugin {

class ButtonProperty : public rviz::Property
{
Q_OBJECT
 public:
  ButtonProperty(const QString& name = QString(), bool default_value = false,
                 const QString& description = QString(), Property* parent = 0,
                 const char *changed_slot = 0, QObject* receiver = 0);

  virtual ~ButtonProperty();

  virtual bool getBool() const;

  //* If this is true, will disable it's children when it's own bool value is false */
  void setDisableChildrenIfFalse(bool disable);

  bool getDisableChildrenIfFalse();

  //* Overridden from Property */
  virtual bool getDisableChildren();

public Q_SLOTS:
  bool setBool( bool value ) { return setValue( value ); }

 private:
  bool disable_children_if_false_;
};

}  // end namespace
