/*
 * BaseShiftTrajectoryBase.hpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <free_gait_core/TypeDefs.hpp>

#include <string>
#include <memory>

namespace free_gait {

/*!
 * Base class for a generic base motions.
 */
class BaseMotionBase
{
 public:

  enum class Type
  {
    Auto,
    Height,
    Target,
    Trajectory,
    Float
  };

  /*!
   * Constructor.
   */
  BaseMotionBase(BaseMotionBase::Type type);

  /*!
   * Destructor.
   */
  virtual ~BaseMotionBase();


  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  virtual std::unique_ptr<BaseMotionBase> clone() const;

  /*!
   * Returns the type of the base motion.
   * @return the type of the base motion.
   */
  BaseMotionBase::Type getType() const;

//  /*!
//   * Update the trajectory with the base start pose.
//   * Do this to avoid jumps of the base.
//   * @param startPose the start pose of the base in the frameId_ frame.
//   * @return true if successful, false otherwise.
//   */
//  virtual bool updateStartPose(const Pose& startPose) = 0;
//
//  /*!
//   * Evaluate the base shift pose at a given time.
//   * @param time the time value.
//   * @return the pose of the base.
//   */
//  virtual const Pose evaluate(const double time) = 0;
//
  /*!
   * Returns the total duration of the motion.
   * @return the duration.
   */
  virtual double getDuration() const;
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
   * @param baseMotion the base motion to debug.
   * @return the resulting output stream.
   */
  friend std::ostream& operator << (std::ostream& out, const BaseMotionBase& baseMotion);

 private:

  //! Type of the base motion.
  Type type_;
//
//  //! Frame id of the base motion.
//  std::string frameId_;
};

} /* namespace */
