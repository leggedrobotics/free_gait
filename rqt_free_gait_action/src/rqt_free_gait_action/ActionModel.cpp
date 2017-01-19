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

#include "rqt_free_gait_action/ActionModel.h"

namespace rqt_free_gait {

/*****************************************************************************/
/** Constructor/Destructor                                                  **/
/*****************************************************************************/

ActionModel::ActionModel(QObject *parent) : QAbstractListModel(parent) {

}

ActionModel::~ActionModel() {

}

/*****************************************************************************/
/** Accessors                                                               **/
/*****************************************************************************/

void ActionModel::addAction(const Action &action) {
  actions_.push_back(action);
  reset();
}

int ActionModel::rowCount(const QModelIndex &parent) const {
  return (int)actions_.size();
}

QVariant ActionModel::data(const QModelIndex &index, int role) const {
  if (!index.isValid()) {
    return QVariant();
  }

  if (role == Qt::TextAlignmentRole) {
    return int(Qt::AlignLeft | Qt::AlignVCenter);
  } else if (role == Qt::DisplayRole) {
    QString name = actions_.at((unsigned long)index.row()).getName();
    return name;
  }

  return QVariant();}

Action ActionModel::getAction(const QModelIndex &index) {
  return actions_.at((unsigned long)index.row());
}

void ActionModel::sortActions() {
  std::sort(actions_.begin(), actions_.end(), ActionModel::comparator);
}

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

bool ActionModel::comparator(const Action &l, const Action &r) {
  return l.getName() < r.getName();
}

} // namespace
