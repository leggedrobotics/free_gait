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

// Quadruped model
#include <quadruped_model/common/enums.hpp>

// STD
#include <unordered_map>

namespace free_gait {

class PoseOptimization
{
 public:
  PoseOptimization();
  virtual ~PoseOptimization();

  typedef std::unordered_map<quadruped_model::LimbEnum, Position, quadruped_model::EnumClassHash> FeetPositions;

  /*!
   * Set the positions of the feet of the robot in world coordinate system.
   * @param feetPositions the feet positions.
   */
  void setFeetPositions(const FeetPositions& feetPositions);

  /*!
   * Define the desired leg configuration by specifying the desired feet positions
   * relative to the base.
   * @param desiredFeetPositionsInBase the desired feet positions in base frame.
   */
  void setDesiredLegConfiguration(const FeetPositions& desiredFeetPositionsInBase);

  /*!
   * Set the support polygon for constraining the pose optimization.
   * If support polygon is not set, the convex hull of all feet positions is used.
   * @param supportPolygon the support polygon as a list of vertices.
   */
  void setSupportPolygon(const grid_map::Polygon& supportPolygon);

  /*!
   * Computes the optimized pose.
   * @param[in/out] optimizedPose
   * @return true if successful, false otherwise.
   */
  bool optimize(Pose& pose);

 private:
  unsigned int nStates_;
  unsigned int nDimensions_;
  FeetPositions feetPositions_;
  FeetPositions desiredFeetPositionsInBase_;
  grid_map::Polygon supportPolygon_;
};

} /* namespace loco */
