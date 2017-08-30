/*
 * PoseOptimizationQP.hpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <free_gait_core/TypeDefs.hpp>

// Grid map
#include <grid_map_core/Polygon.hpp>

// STD
#include <unordered_map>
#include <memory>

// Numerical Optimization
#include <numopt_common/QuadraticProblemSolver.hpp>

namespace free_gait {

class PoseOptimizationQP
{
 public:
  PoseOptimizationQP();
  virtual ~PoseOptimizationQP();
  PoseOptimizationQP(const PoseOptimizationQP& other);

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

  /*!
   * Set the support region for constraining the pose optimization.
   * If support region is not set, the convex hull of all feet positions is used.
   * @param supportPolygon the support polygon as a list of vertices.
   */
  void setSupportRegion(const grid_map::Polygon& supportRegion);

  /*!
   * Computes the optimized pose.
   * @param[in/out] pose the pose to optimize from the given initial guess.
   * @return true if successful, false otherwise.
   */
  bool optimize(Pose& pose);

 private:
  std::unique_ptr<numopt_common::QuadraticProblemSolver> solver_;
  unsigned int nStates_;
  unsigned int nDimensions_;
  Stance stance_;
  Stance nominalStanceInBaseFrame_;
  grid_map::Polygon supportRegion_;
};

} /* namespace loco */
