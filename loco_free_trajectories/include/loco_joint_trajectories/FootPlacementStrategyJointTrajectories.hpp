/*
 * FootPlacementStrategyJointTrajectories.hpp
 *
 *  Created on: March 14, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Loco
#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"
#include "loco/common/TypeDefs.hpp"
#include "loco/common/TorsoBase.hpp"
#include "loco/common/LegGroup.hpp"
#include "loco/common/StepQueue.hpp"

// TinyXML
#include "tinyxml.h"

// Eigen
#include <Eigen/Core>

// Kindr
#include "kindr/rotations/RotationEigen.hpp"

#include "robotUtils/function_approximators/polyharmonicSplines/BoundedRBF1D.hpp"

namespace loco {

class FootPlacementStrategyJointTrajectories: public FootPlacementStrategyBase {

public:
  FootPlacementStrategyJointTrajectories(std::shared_ptr<LegGroup> legs,
                                         std::shared_ptr<TorsoBase> torso,
                                         std::shared_ptr<StepQueue> stepQueue);
  virtual ~FootPlacementStrategyJointTrajectories();

  virtual bool loadParameters(const TiXmlHandle& handle);
  virtual bool initialize(double dt);
  virtual bool advance(double dt);

  void generateDesiredFootPosition(LegBase* leg) const;

  /*! Computes an interpolated version of the two controllers passed in as parameters.
  *  If t is 0, the current setting is set to footPlacementStrategy1, 1 -> footPlacementStrategy2, and values in between
  *  correspond to interpolated parameter set.
  * @param footPlacementStrategy1
  * @param footPlacementStrategy2
  * @param t interpolation parameter
  * @returns true if successful
  */
  virtual bool setToInterpolated(const FootPlacementStrategyBase& footPlacementStrategy1, const FootPlacementStrategyBase& footPlacementStrategy2, double t);
  const LegGroup& getLegs() const;

private:
  //! True if initialized, false otherwise.
  bool isInitialized_;

  //! Legs information container.
  std::shared_ptr<LegGroup> legs_;

  //! Torso information container.
  std::shared_ptr<TorsoBase> torso_;

  //! Step Queue information container.
  std::shared_ptr<StepQueue> stepQueue_;

  virtual void setJointPositions(LegBase* leg, const Position& footPositionInWorldFrame) const;
};

} /* namespace loco */
