/******************************************************************************
 * Copyright (C) 2014 by Samuel Bachmann                                      *
 * samuel.bachmann@gmail.com                                                  *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include <iostream>

#include "rqt_free_gait_action/Collection.h"

namespace rqt_free_gait {

/*****************************************************************************/
/** Constructor/Destructor                                                  **/
/*****************************************************************************/

Collection::Collection(QString id, QString name, ActionModel *actionModel)
    : id_(id),
      name_(name),
      actionModel_(actionModel) {

}

Collection::~Collection() {

}

/*****************************************************************************/
/** Accessors                                                               **/
/*****************************************************************************/

const QString &Collection::getId() const {
  return id_;
}

const QString &Collection::getName() const {
  return name_;
}

ActionModel *Collection::getActionModel() {
  return actionModel_;
}

} // namespace
