/*
 * GaitPatternJointTrajectories.hpp
 *
 *  Created on: Jan 14, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "tinyxml.h"

// STL
#include <memory>
#include <vector>

// Loco
#include "loco/gait_pattern/GaitPatternBase.hpp"
#include "loco/common/TorsoBase.hpp"
#include "loco/common/LegGroup.hpp"
#include "loco/common/StepQueue.hpp"

namespace loco {

class GaitPatternJointTrajectories: public GaitPatternBase {
public:
  GaitPatternJointTrajectories(std::shared_ptr<LegGroup> legs, std::shared_ptr<TorsoBase> torso,
                      std::shared_ptr<StepQueue> stepQueue);

  virtual ~GaitPatternJointTrajectories();

  /*! Loads the parameters from the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  virtual bool loadParameters(const TiXmlHandle &hParameterSet);

  /*! Stores the current parameters in the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  virtual bool saveParameters(TiXmlHandle &hParameterSet);

  virtual bool initialize(double dt);

  bool isInitialized();

  /*! @returns the relative phase for the leg whose index is passed in. The number
    returned is always going to be between 0 and 1 (0 meaning it should still be in stance mode,
    1 - it is a stance leg again, anything in between means that it is a swing leg).
    The stridePhase is expected to be between 0 and 1.
  */
  virtual double getSwingPhaseForLeg(int iLeg);

  /*! @returns the relative phase for the leg whose index is passed in. The number
    returned is always going to be between 0 and 1 (0 meaning it should still be in stance mode,
    1 - it is a stance leg again, anything in between means that it is a swing leg).
    The stridePhase is expected to be between 0 and 1.
  */
  virtual double getSwingPhaseForLeg(int iLeg, double stridePhase) const;

  //! returns the relative stance phase for the leg. If the leg is in swing mode, it returns -1
  virtual double getStancePhaseForLeg(int iLeg);

  /*!  @returns the relative stance phase for the leg. If the limb is in swing mode, it returns -1
  */
  virtual double getStancePhaseForLeg(int iLeg, double stridePhase) const;

  //! @returns the total length of the stance phase in seconds
  virtual double getStanceDuration(int iLeg);

  //! @returns the total length of the swing phase in seconds, stride duration is ignored
  virtual double getSwingDuration(int iLeg, double strideDuration = 0.0) const;

  //! @returns the total length of the stance phase in seconds for a a given stride duration
  virtual double getStanceDuration(int iLeg, double strideDuration) const;


  virtual double getStrideDuration();
  virtual void setStrideDuration(double strideDuration);

  virtual unsigned long int getNGaitCycles();


  void setVelocity(double value);

  double getVelocity();

  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual bool advance(double dt);

  virtual bool shouldBeLegGrounded(int iLeg);

  /*! @returns stride (cycle) phase, which is between [0, 1].
   */
  virtual double getStridePhase() const;

  /*! Sets the stride (cycle phase), which is between [0, 1].
   * @param stridePhase cycle phase
   */
  virtual void setStridePhase(double stridePhase);


  /*! @returns the time left in stance in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeLeftInStance(int iLeg, double strideDuration, double stridePhase) const;

  /*! @returns the time left in swing in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeLeftInSwing(int iLeg, double strideDuration, double stridePhase) const;

  /*! @returns the time spent in stance in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeSpentInStance(int iLeg, double strideDuration, double stridePhase) const;

  /*! @returns the time spent in swing in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeSpentInSwing(int iLeg, double strideDuration, double stridePhase) const;

  /*! @returns time until next stance phase starts in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the stride
   */
  virtual double getTimeUntilNextStancePhase(int iLeg, double strideDuration, double stridePhase) const;

  /*! @returns time until next swing phase starts in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the stride
   */
  virtual double getTimeUntilNextSwingPhase(int iLeg, double strideDuration, double stridePhase) const;


  /*! @returns number of legs that are in stance mode
   * @param stridePhase   stride phase
   */
  virtual int getNumberOfStanceLegs(double stridePhase);

private:

  //! True if initialized, false otherwise.
  bool isInitialized_;

  //! Legs information container.
  std::shared_ptr<LegGroup> legs_;

  //! Torso information container.
  std::shared_ptr<TorsoBase> torso_;

  //! Step Queue information container.
  std::shared_ptr<StepQueue> stepQueue_;
};

} // namespace loco

