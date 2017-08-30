/******************************************************************************
 * Copyright 2017 Samuel Bachmann, Peter Fankhauser                           *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions are met:*
 *                                                                            *
 * 1. Redistributions of source code must retain the above copyright notice,  *
 * this list of conditions and the following disclaimer.                      *
 *                                                                            *
 * 2. Redistributions in binary form must reproduce the above copyright       *
 * notice, this list of conditions and the following disclaimer in the        *
 * documentation and/or other materials provided with the distribution.       *
 *                                                                            *
 * 3. Neither the name of the copyright holder nor the names of its           *
 * contributors may be used to endorse or promote products derived from this  *
 * software without specific prior written permission.                        *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE  *
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR        *
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF       *
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS   *
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN    *
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)    *
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF     *
 * THE POSSIBILITY OF SUCH DAMAGE.                                            *
 *                                                                            *
 * Authors: Samuel Bachmann <samuel.bachmann@gmail.com>,                      *
 *          Peter Fankhauser <pfankhauser@ethz.ch>                            *
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

#if QT_VERSION >= QT_VERSION_CHECK(5,0,0)
  // TODO reset model?
//  beginResetModel();
//  myData.clear();
//  endResetModel();
#else
  reset();
#endif
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

std::vector<Action> CollectionModel::getActions(QString collectionId) {
  for (int i = 0; i < collections_.size(); ++i) {
    if (collections_[i].getId().compare(collectionId) == 0) {
      return collections_[i].getActionModel()->getActions();
    }
  }
  return std::vector<Action>();
}

QString CollectionModel::getCollectionId(const QModelIndex &index) {
  return collections_.at((unsigned long)index.row()).getId();
}

bool CollectionModel::isSequence(const QModelIndex &index) {
  return collections_.at((unsigned long)index.row()).isSequence();
}

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

bool CollectionModel::comparator(const Collection &l, const Collection &r) {
  return l.getName() < r.getName();
}

} // namespace
