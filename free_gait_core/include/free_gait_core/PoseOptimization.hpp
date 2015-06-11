/*
 * PoseOptimization.hpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <free_gait_core/TypeDefs.hpp>

// Grid map
#include <grid_map_core/Polygon.hpp>

namespace free_gait {

class PoseOptimization
{
 public:
  PoseOptimization();
  virtual ~PoseOptimization();

  /*!
   * Set the positions of the feet of the robot in world coordinate system.
   * @param feetPositions the feet positions.
   */
  void setFeetPositions(const std::vector<Position>& feetPositions);

  /*!
   * Define the desired leg configuration by specifying the desired feet positions
   * relative to the base.
   * Important: The number and order of the feet has to be the same as in `setFeetPositions`.
   * @param desiredFeetPositionsInBase the desired feet positions in base frame.
   */
  void setDesiredLegConfiguration(const std::vector<Position>& desiredFeetPositionsInBase);

  /*!
   * Set the support polygon for constraining the pose optimization.
   * If support polygon is not set, the convex hull of all feet positions is used.
   * @param supportPolygon the support polygon as a list of vertices.
   */
  void setSupportPolygon(const grid_map::Polygon& supportPolygon);

  /*!
   * Set the start pose of the base as initial guess for the optimization.
   * @param startPose the start pose of the base.
   */
  void setStartPose(const Pose& startPose);

  /*!
   * Computes the optimized pose.
   * @param[out] optimizedPose
   * @return true if successful, false otherwise.
   */
  bool compute(Pose& optimizedPose);

 private:
  unsigned int nStates_;
  std::vector<Position> feetPositions_;
  std::vector<Position> desiredFeetPositionsInBase_;
  grid_map::Polygon supportPolygon_;
  double safetyMargin_;
  Pose startPose_;
};

} /* namespace loco */
