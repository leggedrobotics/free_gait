/*
 * SwingData.hpp
 *
 *  Created on: March 5, 2015
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
#include "free_gait_core/SwingTrajectoryBase.hpp"
#include "free_gait_core/SwingProfile.hpp"

namespace free_gait {

class SwingData
{
 public:
  SwingData();
  virtual ~SwingData();

  /*!
   * Copy-constructor.
   * @param other the swing data to be copied from.
   */
  SwingData(const SwingData& other);

  const std::string& getName() const;
  void setName(const std::string& name);

  const Vector& getSurfaceNormal() const;
  void setSurfaceNormal(const Vector& surfaceNormal);

  std::shared_ptr<SwingTrajectoryBase> getTrajectory();
  const SwingTrajectoryBase& getTrajectory() const;
  void setTrajectory(const SwingTrajectoryBase& trajectory);

  bool isUsingProfile() const;
  void setUseProfile(bool useProfile);

  friend std::ostream& operator << (std::ostream& out, const SwingData& swingData);


 protected:
  std::string name_;

  free_gait::Vector surfaceNormal_;

  std::shared_ptr<SwingTrajectoryBase> trajectory_;

 private:
  bool useProfile_;
};

} /* namespace loco */
