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

enum class BaseShiftTrajectoryType
{
  Profile,
  Trajectory,
  ZMP
};

/*!
 * Base class for a generic base shift trajectory.
 */
class BaseShiftTrajectoryBase
{
 public:

  /*!
   * Constructor.
   */
  BaseShiftTrajectoryBase(BaseShiftTrajectoryType type);

  /*!
   * Destructor.
   */
  virtual ~BaseShiftTrajectoryBase();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  virtual std::unique_ptr<BaseShiftTrajectoryBase> clone() const = 0;

  /*!
   * Returns the type of the base shift trajectory.
   * @return the type of the base shift trajectory.
   */
  virtual BaseShiftTrajectoryType getType() const;

  /*!
   * Update the trajectory with the base start pose.
   * Do this to avoid jumps of the base.
   * @param startPose the start pose of the base in the frameId_ frame.
   * @return true if successful, false otherwise.
   */
  virtual bool updateStartPose(const Pose& startPose) = 0;

  /*!
   * Evaluate the base shift pose at a given time.
   * @param time the time value.
   * @return the pose of the base.
   */
  virtual const Pose evaluate(const double time) = 0;

  /*!
   * Returns the total duration of the trajectory.
   * @return the duration.
   */
  virtual double getDuration() const = 0;

  /*!
   * Returns the frame id of the trajectory.
   * @return the frame id.
   */
  const std::string& getFrameId() const;

  /*!
   * Set the frame id of the trajectory.
   * @param frameId the frame id.
   */
  void setFrameId(const std::string& frameId);

  /*!
   * Print the contents to console for debugging.
   * @param out the output stream.
   * @param baseShiftTrajectory the base shift trajectory to debug.
   * @return the resulting output stream.
   */
  friend std::ostream& operator << (std::ostream& out, const BaseShiftTrajectoryBase& baseShiftTrajectory);

 protected:

  //! Type of the trajectory.
  BaseShiftTrajectoryType type_;

  //! Frame id of the trajectory.
  std::string frameId_;
};

} /* namespace */
