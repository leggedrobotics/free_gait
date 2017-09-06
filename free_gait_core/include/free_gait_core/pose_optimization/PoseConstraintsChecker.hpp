/*
 * PoseConstraintsChecker.hpp
 *
 *  Created on: Aug 31, 2017
 *      Author: Péter Fankhauser
 */

#pragma once

#include "free_gait_core/pose_optimization/PoseOptimizationBase.hpp"
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/executor/AdapterBase.hpp"

#include <grid_map_core/Polygon.hpp>

namespace free_gait {

class PoseConstraintsChecker : public PoseOptimizationBase
{
 public:
  PoseConstraintsChecker(const AdapterBase& adapter);
  virtual ~PoseConstraintsChecker();

  void setTolerances(const double centerOfMassTolerance, const double legLengthTolerance);

  bool check(const Pose& pose);

 private:
  double centerOfMassTolerance_;
  double legLengthTolerance_;
};

}
