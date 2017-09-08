/*
 * PoseOptimizationBase.hpp
 *
 *  Created on: Aug 31, 2017
 *      Author: Péter Fankhauser
 */

#pragma once

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/executor/AdapterBase.hpp"

#include <grid_map_core/Polygon.hpp>

namespace free_gait {

class PoseOptimizationBase
{
 public:
  PoseOptimizationBase(const AdapterBase& adapter);
  virtual ~PoseOptimizationBase();

  typedef std::map<LimbEnum, double> LimbLengths;

  /*!
   * Set the current state of the robot.
   * @param state the current state.
   */
  virtual void setCurrentState(const State& state);

  /*!
   * Set the positions of the feet of the robot in world coordinate system.
   * @param feetPositions the feet positions.
   */
  virtual void setStance(const Stance& stance);

  virtual void setSupportStance(const Stance& supportStance);

  /*!
   * Define the desired leg configuration by specifying the desired feet positions
   * relative to the base.
   * @param nominalStanceInBaseFrame the desired feet positions in base frame.
   */
  virtual void setNominalStance(const Stance& nominalStanceInBaseFrame);

  /*!
   * Set the support polygon for constraining the pose optimization.
   * If support polygon is not set, the convex hull of the support feet positions (support stance) is used.
   * @param supportPolygon the support polygon as a list of vertices.
   */
  virtual void setSupportRegion(const grid_map::Polygon& supportRegion);

  /*!
   * Set the max. and min. value for constraining the limb length.
   * Note: minLimbLenghts and maxLimbLenghts need to have same length!
   * @param minLimbLenghts the min. limb length.
   * @param maxLimbLenghts the max. limb length.
   */
  virtual void setLimbLengthConstraints(const LimbLengths& minLimbLenghts, const LimbLengths& maxLimbLenghts);

 protected:

  /*!
   * Check if support region is set. If not, use convex hull of stance.
   */
  virtual void checkSupportRegion();

  /*!
   * Sets the joint positions based on the pose in the state and the provided stance.
   * @param[in/out] state the state to update the joints for
   * @return true if all legs were updated, false if IK could not be solved.
   */
  virtual bool updateJointPositionsInState(State& state) const;

  const AdapterBase& adapter_;
  State state_;
  Stance stance_;
  Stance supportStance_;
  Stance nominalStanceInBaseFrame_;
  grid_map::Polygon supportRegion_;
  LimbLengths minLimbLenghts_, maxLimbLenghts_;
};

}
