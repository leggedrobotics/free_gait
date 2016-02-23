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

std::ostream& operator<< (std::ostream& out, const ControlSetup& controlSetup)
{
  for (const auto& controlLevel : controlSetup) {
    if (controlLevel.second) out << controlLevel.first << ", ";
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

std::ostream& operator<< (std::ostream& out, const BranchEnum& branch)
{
  switch (branch) {
    case BranchEnum::BASE:
      out << "BASE";
      return out;
    case BranchEnum::LF_LEG:
      out << "LF_LEG";
      return out;
    case BranchEnum::RF_LEG:
      out << "RF_LEG";
      return out;
    case BranchEnum::LH_LEG:
      out << "LH_LEG";
      return out;
    case BranchEnum::RH_LEG:
      out << "RH_LEG";
      return out;
    default:
      out << "Undefined";
      return out;
  }
}

//std::ostream& operator<< (std::ostream& out, const bool boolean)
//{
//  switch (boolean) {
//    case true:
//      out << "True";
//      return out;
//    case false:
//      out << "False";
//      return out;
//    default:
//      out << "Undefined";
//      return out;
//  }
//}

std::ostream& operator<< (std::ostream& out, const std::unordered_map<LimbEnum, bool, EnumClassHash>& limbBooleanMap)
{
  for (const auto& limb : limbBooleanMap) {
    out << limb.first << ": " << limb.second << ", ";
  }
  return out;
}


} // namespace
