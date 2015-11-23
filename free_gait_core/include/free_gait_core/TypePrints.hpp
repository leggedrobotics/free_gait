/*
 * TypePrints.cpp
 *
 *  Created on: Nov 19, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/TypeDefs.hpp"

namespace free_gait {

std::ostream& operator<< (std::ostream& out, const ControlLevel& controlLevel);
std::ostream& operator<< (std::ostream& out, const LimbEnum& limb);

} // namespace
