/*
 * Geometric.hpp
 *
 *  Created on: Aug 30, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include <free_gait_core/TypeDefs.hpp>

#include <grid_map_core/Polygon.hpp>

#include <Eigen/Eigenvalues>
#include <memory>
#include <unordered_map>

namespace free_gait {

class PoseOptimizationGeometric
{
 public:
  PoseOptimizationGeometric();
  virtual ~PoseOptimizationGeometric();

  /*!
   * Computes the optimized pose.
   * @param stance the positions of the feet of the robot in world coordinate system.
   * @param nominalStanceInBaseFrame the desired feet positions in base frame.
   * @param supportRegion the support region as a list of vertices
   * @return the optimized pose.
   */
  static const Pose optimize(const Stance& stance, const Stance& nominalStanceInBaseFrame,
                             const grid_map::Polygon& supportRegion = grid_map::Polygon());
};

} /* namespace loco */
