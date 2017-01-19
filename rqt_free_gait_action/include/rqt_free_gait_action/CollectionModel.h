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

#pragma once

#include <algorithm>
#include <vector>

#include <QAbstractListModel>

#include "rqt_free_gait_action/Collection.h"

namespace rqt_free_gait {

class CollectionModel : public QAbstractListModel {
public:

  /***************************************************************************/
  /** Constructor/Destructor                                                **/
  /***************************************************************************/

  CollectionModel(QObject *parent = 0);

  ~CollectionModel();

  /***************************************************************************/
  /** Accessors                                                             **/
  /***************************************************************************/

  void addCollection(const Collection &collection);

  int rowCount(const QModelIndex &parent) const;

  QVariant data(const QModelIndex &index, int role) const;

  ActionModel *getActionModel(const QModelIndex &index);

  void sortCollections();

private:

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  std::vector<Collection> collections_;

  /***************************************************************************/
  /** Methods                                                               **/
  /***************************************************************************/

  static bool comparator(const Collection &l, const Collection &r);

};

} // namespace
