/*
 * PoseOptimizationFunctionConstraints.hpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include <numopt_common/NonlinearFunctionConstraints.hpp>
#include <grid_map_core/Polygon.hpp>

namespace free_gait {

class PoseOptimizationFunctionConstraints : public numopt_common::NonlinearFunctionConstraints
{
 public:
  PoseOptimizationFunctionConstraints();
  virtual ~PoseOptimizationFunctionConstraints();

  void setSupportRegion(const grid_map::Polygon& supportRegion);
  const grid_map::Polygon& getSupportRegion() const;

  bool getGlobalBoundConstraintMinValues(numopt_common::Vector& values);
  bool getGlobalBoundConstraintMaxValues(numopt_common::Vector& values);

  //! d <= c(p) <= f
  bool getInequalityConstraintValues(numopt_common::Vector& values, const numopt_common::Parameterization& p, bool newParams = true);
  bool getInequalityConstraintMinValues(numopt_common::Vector& d);
  bool getInequalityConstraintMaxValues(numopt_common::Vector& f);

 private:
  grid_map::Polygon supportRegion_;
  //! A from A*x <= b.
  Eigen::MatrixXd supportRegionInequalityConstraintGlobalJacobian_;
  //! b from A*x <= b.
  numopt_common::Vector supportRegionInequalityConstraintsMaxValues_;
};

} /* namespace */
