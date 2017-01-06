/*
 * TypeDefs.cpp
 *
 *  Created on: Jan 6, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "free_gait_core/TypeDefs.hpp"

namespace free_gait {

struct CompareByCounterClockwiseOrder
{
  bool operator()(const LimbEnum& a, const LimbEnum& b) const
  {
    const size_t aIndex = findIndex(a);
    const size_t bIndex = findIndex(b);
    return aIndex < bIndex;
  }

  size_t findIndex(const LimbEnum& limb) const
  {
    auto it = std::find(limbEnumCounterClockWiseOrder.begin(), limbEnumCounterClockWiseOrder.end(), limb);
    if (it == limbEnumCounterClockWiseOrder.end()) {
      throw std::runtime_error("Could not find index for limb!");
    }
    return std::distance(limbEnumCounterClockWiseOrder.begin(), it);
  }
};

void getFootholdsCounterClockwiseOrdered(const Stance& stance, std::vector<Position>& footholds)
{
  footholds.reserve(stance.size());
  std::map<LimbEnum, Position, CompareByCounterClockwiseOrder> stanceOrdered;
  for (const auto& limb : stance) stanceOrdered.insert(limb);
  for (const auto& limb : stanceOrdered) footholds.push_back(limb.second);
};

} // namespace
