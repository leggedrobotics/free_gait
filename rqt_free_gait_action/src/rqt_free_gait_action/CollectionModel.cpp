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

#include "rqt_free_gait_action/CollectionModel.h"

namespace rqt_free_gait {

/*****************************************************************************/
/** Constructor/Destructor                                                  **/
/*****************************************************************************/

CollectionModel::CollectionModel(QObject *parent) : QAbstractListModel(parent) {

}

CollectionModel::~CollectionModel() {

}

/*****************************************************************************/
/** Accessors                                                               **/
/*****************************************************************************/

void CollectionModel::addCollection(const Collection &collection) {
  collections_.push_back(collection);
  reset();
}

int CollectionModel::rowCount(const QModelIndex &/*parent*/) const {
  return (int)collections_.size();
}

QVariant CollectionModel::data(const QModelIndex &index, int role) const {
  if (!index.isValid()) {
    return QVariant();
  }

  if (role == Qt::TextAlignmentRole) {
    return int(Qt::AlignLeft | Qt::AlignVCenter);
  } else if (role == Qt::DisplayRole) {
    return collections_.at((unsigned long)index.row()).getName();
  }

  return QVariant();
}

ActionModel *CollectionModel::getActionModel(
    const QModelIndex &index) {
  return collections_.at((unsigned long)index.row()).getActionModel();
}

void CollectionModel::sortCollections() {
  std::sort(collections_.begin(), collections_.end(),
            CollectionModel::comparator);
}

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

bool CollectionModel::comparator(const Collection &l, const Collection &r) {
  return l.getName() < r.getName();
}

} // namespace
