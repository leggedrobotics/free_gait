/*
 * PoseOptimizationObjectiveFunction.hpp
 *
 *  Created on: Mar 22, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include "free_gait_core/executor/AdapterBase.hpp"
#include "free_gait_core/executor/State.hpp"

#include <numopt_common/NonlinearObjectiveFunction.hpp>

#include <memory>

namespace free_gait {

class PoseOptimizationObjectiveFunction : public numopt_common::NonlinearObjectiveFunction
{
 public:
  /*!
   * Initialize the pose optimization objective function.
   * Note: The robot model behind the adapter is internally changed, make sure to give it
   * an adapter only used for this optimization.
   * @param adapter the Free Gait adapter.
   */
  PoseOptimizationObjectiveFunction(const AdapterBase& adapter, const State& state);
  virtual ~PoseOptimizationObjectiveFunction();

  /*!
   * Set the positions of the feet (stance) of the robot in world coordinate system.
   * @param stance the feet positions.
   */
  void setStance(const Stance& stance);

  /*!
   * Define the desired leg configuration by specifying the desired feet positions
   * relative to the base.
   * @param nominalStanceInBaseFrame the desired feet positions in base frame.
   */
  void setNominalStance(const Stance& nominalStanceInBaseFrame);

  /*! This method computes the objective value
   * @param value       function value
   * @param p           parameters
   * @param newParams   true if this class has already seen the parameters
   * @return  true if successful
   */
  bool computeValue(numopt_common::Scalar& value, const numopt_common::Parameterization& params,
                    bool newParams = true);


  /*! Computes the local gradient of the objective function.
   * @param gradient    gradient (column vector)
   * @param p           parameters
   * @param newParams   true if this class has already seen the parameters
   * @returns true if successful
   */
  bool getLocalGradient(numopt_common::Vector& gradient, const numopt_common::Parameterization& p,
                        bool newParams = true);

 private:
  const AdapterBase& adapter_;
  const State& state_;

  Stance stance_;
  Stance nominalStanceInBaseFrame_;

//  //! Hip to foot length along world z-axis.
//  const double desiredLegVerticalLength_;
};

} /* namespace free_gait */
