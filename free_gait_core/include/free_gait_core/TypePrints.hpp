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
std::ostream& operator<< (std::ostream& out, const ControlSetup& controlSetup);
std::ostream& operator<< (std::ostream& out, const LimbEnum& limb);
std::ostream& operator<< (std::ostream& out, const BranchEnum& branch);
//std::ostream& operator<< (std::ostream& out, const bool boolean);
std::ostream& operator<< (std::ostream& out, const std::unordered_map<LimbEnum, bool, EnumClassHash>& limbBooleanMap);
std::ostream& operator<< (std::ostream& out, const std::unordered_map<LimbEnum, Vector, EnumClassHash>& limbVectorMap);
std::ostream& operator<< (std::ostream& out, const Stance& stance);

} // namespace
