/*
 * PoseOptimizationFunctionConstraints.hpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include <numopt_common/NonlinearFunctionConstraints.hpp>

namespace free_gait {

class PoseOptimizationFunctionConstraints : public numopt_common::NonlinearFunctionConstraints
{
 public:
  PoseOptimizationFunctionConstraints();
  virtual ~PoseOptimizationFunctionConstraints();

  bool getGlobalBoundConstraintMinValues(numopt_common::Vector& values);
  bool getGlobalBoundConstraintMaxValues(numopt_common::Vector& values);

};

} /* namespace */
