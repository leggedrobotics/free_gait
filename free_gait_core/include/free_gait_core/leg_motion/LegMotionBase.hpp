/*
 * LegMotionBase.hpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include <free_gait_core/TypeDefs.hpp>

// STD
#include <string>
#include <memory>

namespace free_gait {

/*!
 * Base class for a generic swing leg motion.
 */
class LegMotionBase
{
 public:

  enum class Type
  {
    Footstep,
    EndEffectorTarget,
    EndEffectorTrajectory,
    JointTarget,
    JointTrajectory
  };

  /*!
   * Constructor.
   */
  LegMotionBase(LegMotionBase::Type type);

  /*!
   * Destructor.
   */
  virtual ~LegMotionBase();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  virtual std::unique_ptr<LegMotionBase> clone() const;

  /*!
   * Returns the type of the swing trajectory.
   * @return the type of the swing trajectory.
   */
  virtual LegMotionBase::Type getType() const;

//  /*!
//   * Update the trajectory with the foot start position.
//   * Do this to avoid jumps of the swing leg.
//   * @param footStartPosition the start position of the foot in the trajectoryFrameId_ frame.
//   * @return true if successful, false otherwise.
//   */
//  virtual bool updateStartPosition(const Position& startPosition) = 0;
//
//  /*!
//   * Evaluate the swing foot position at a given swing phase value.
//   * @param phase the swing phase value.
//   * @return the position of the foot on the swing trajectory.
//   */
//  virtual const Position evaluate(const double phase) = 0;
//
  /*!
   * Returns the total duration of the motion.
   * @return the duration.
   */
  virtual double getDuration() const;
//
//  /*!
//   * Return the target (end position) of the swing trajectory.
//   * @return the target.
//   */
//  virtual const Position getTarget() const = 0;
//
//  /*!
//   * Returns the frame id of the trajectory.
//   * @return the frame id.
//   */
//  const std::string& getFrameId() const;
//
//  /*!
//   * Set the frame id of the trajectory.
//   * @param frameId the frame id.
//   */
//  void setFrameId(const std::string& frameId);

  /*!
   * Print the contents to console for debugging.
   * @param out the output stream.
   * @param swingTrajectory the swing trajectory to debug.
   * @return the resulting output stream.
   */
  friend std::ostream& operator << (std::ostream& out, const LegMotionBase& legMotion);

 private:

  //! Type of the leg motion.
  Type type_;
};

} /* namespace */
