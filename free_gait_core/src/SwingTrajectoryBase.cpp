/*
 * SwingTrajectoryBase.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/SwingTrajectoryBase.hpp>

namespace free_gait {

SwingTrajectoryBase::SwingTrajectoryBase()
{
  // TODO Auto-generated constructor stub

}

SwingTrajectoryBase::~SwingTrajectoryBase()
{
  // TODO Auto-generated destructor stub
}

const std::string& SwingTrajectoryBase::getFrameId() const
{
  return frameId_;
}

void SwingTrajectoryBase::setFrameId(const std::string& frameId)
{
  frameId_ = frameId;
}

} /* namespace loco */
