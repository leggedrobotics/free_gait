/*
 * PoseOptimizationFunctionConstraints.hpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/executor/AdapterBase.hpp"
#include "free_gait_core/executor/State.hpp"

#include <numopt_common/NonlinearFunctionConstraints.hpp>
#include <grid_map_core/Polygon.hpp>

namespace free_gait {

class PoseOptimizationFunctionConstraints : public numopt_common::NonlinearFunctionConstraints
{
 public:
  typedef std::map<LimbEnum, double> LegLengths;

  PoseOptimizationFunctionConstraints(const AdapterBase& adapter, const State& state);
  virtual ~PoseOptimizationFunctionConstraints();

  void setStance(const Stance& stance);
  void setSupportRegion(const grid_map::Polygon& supportRegion);
  const grid_map::Polygon& getSupportRegion() const;

  /*!
   * Note: minLimbLenghts and maxLimbLenghts need to have same length!
   * @param minLimbLenghts
   * @param maxLimbLenghts
   */
  void setLimbLengthConstraints(const LegLengths& minLimbLenghts,
                                const LegLengths& maxLimbLenghts);

  bool getGlobalBoundConstraintMinValues(numopt_common::Vector& values);
  bool getGlobalBoundConstraintMaxValues(numopt_common::Vector& values);

  //! d <= c(p) <= f
  bool getInequalityConstraintValues(numopt_common::Vector& values, const numopt_common::Parameterization& p, bool newParams = true);
  bool getInequalityConstraintMinValues(numopt_common::Vector& d);
  bool getInequalityConstraintMaxValues(numopt_common::Vector& f);

 private:
  void updateNumberOfInequalityConstraints();

  const AdapterBase& adapter_;
  const State& state_;

  Stance stance_;

  size_t nSupportRegionInequalityConstraints_;
  grid_map::Polygon supportRegion_;

  size_t nLimbLengthInequalityConstraints_;
  numopt_common::Vector limbLengthInequalityConstraintsMinValues_;
  numopt_common::Vector limbLengthInequalityConstraintsMaxValues_;

  //! A from A*x <= b.
  Eigen::MatrixXd supportRegionInequalityConstraintGlobalJacobian_;
  //! b from A*x <= b.
  numopt_common::Vector supportRegionInequalityConstraintsMaxValues_;
};

} /* namespace */
