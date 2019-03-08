/*
 * Geometric.hpp
 *
 *  Created on: Aug 30, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include "free_gait_core/pose_optimization/PoseOptimizationBase.hpp"
#include "free_gait_core/TypeDefs.hpp"

namespace free_gait {

class PoseOptimizationGeometric : public PoseOptimizationBase
{
 public:
  PoseOptimizationGeometric(const AdapterBase& adapter);
  virtual ~PoseOptimizationGeometric();

  /*!
   * Set the positions of the feet of the robot in world coordinate
   * system for the computation of the orientation.
   * @param stance the position to determine the orientation.
   */
  void setStanceForOrientation(const Stance& stance);

  /*!
   * Computes the optimized pose.
   * @param stanceForOrientation the positions of the feet of the robot in world coordinate
   *        system for the computation of the orientation.
   * @return the optimized pose.
   */
  bool optimize(Pose& pose);

  void setNominalRotation(const RotationQuaternion& rotation);

 private:
  Stance stanceForOrientation_;
  RotationQuaternion nominalRotation_;
  bool hasNominalRotation_;
};

} /* namespace loco */
