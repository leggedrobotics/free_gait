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

#include <QString>

namespace rqt_free_gait {

class Action {
public:

  /***************************************************************************/
  /** Constructor/Destructor                                                **/
  /***************************************************************************/

  Action(QString id, QString name, QString description);

  ~Action();

  /***************************************************************************/
  /** Accessors                                                             **/
  /***************************************************************************/

  const QString& getId() const;

  const QString& getName() const;

  const QString& getDescription() const;

private:

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  QString id_;
  QString name_;
  QString description_;
};

} // namespace
