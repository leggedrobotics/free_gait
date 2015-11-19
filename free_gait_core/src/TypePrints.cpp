/*
 * TypePrints.hpp
 *
 *  Created on: Nov 19, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/TypePrints.hpp"

namespace free_gait {

std::ostream& operator<< (std::ostream& out, const ControlLevel& controlLevel)
{
  switch (controlLevel) {
    case ControlLevel::Position:
      out << "Position";
      return out;
    case ControlLevel::Velocity:
      out << "Velocity";
      return out;
    case ControlLevel::Acceleration:
      out << "Acceleration";
      return out;
    case ControlLevel::Effort:
      out << "Effort";
      return out;
    default:
      out << "Undefined";
      return out;
  }
}

std::ostream& operator<< (std::ostream& out, const LimbEnum& limb)
{
  switch (limb) {
    case LimbEnum::LF_LEG:
      out << "LF_LEG";
      return out;
    case LimbEnum::RF_LEG:
      out << "RF_LEG";
      return out;
    case LimbEnum::LH_LEG:
      out << "LH_LEG";
      return out;
    case LimbEnum::RH_LEG:
      out << "RH_LEG";
      return out;
    default:
      out << "Undefined";
      return out;
  }
}

} // namespace
