/*
 * BaseShiftData.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// STL
#include <string>
#include <iostream>
#include <memory>

// Free Gait
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/BaseShiftTrajectoryBase.hpp"

namespace free_gait {

class BaseShiftData
{
 public:
  BaseShiftData();
  virtual ~BaseShiftData();

  /*!
   * Copy-constructor.
   * @param other the swing data to be copied from.
   */
  BaseShiftData(const BaseShiftData& other);

  const std::string& getName() const;
  void setName(const std::string& name);
  
  bool isIgnore() const;
  void setIgnore(bool ignore);

  std::shared_ptr<BaseShiftTrajectoryBase> getTrajectory();
  const BaseShiftTrajectoryBase& getTrajectory() const;
  void setTrajectory(const BaseShiftTrajectoryBase& trajectory);

  friend std::ostream& operator << (std::ostream& out, const BaseShiftData& baseShiftData);

 protected:
  std::string name_;
  bool ignore_;
  std::shared_ptr<BaseShiftTrajectoryBase> trajectory_;
};

} /* namespace */
