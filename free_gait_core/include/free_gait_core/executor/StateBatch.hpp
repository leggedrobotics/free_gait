/*
 * StateBatch.hpp
 *
 *  Created on: Dec 20, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <free_gait_core/executor/State.hpp>

#include <map>
#include <string>
#include <tuple>

namespace free_gait {

class StateBatch
{
 public:
  StateBatch();
  virtual ~StateBatch();

  const std::map<double, State>& getStates() const;
  std::vector<std::map<double, Position>> getEndEffectorPositions() const;
  std::vector<std::map<double, Position>> getEndEffectorTargets() const;
  std::vector<std::map<double, std::tuple<Position, Vector>>> getSurfaceNormals() const;
  std::map<double, Stance> getStances() const;
  bool getEndTimeOfStep(std::string& stepId, double& endTime) const;
  void addState(const double time, const State& state);
  bool isValidTime(const double time) const;
  double getStartTime() const;
  double getEndTime() const;
  const State& getState(const double time) const;
  void clear();

  friend class StateBatchComputer;

 private:
  std::map<double, State> states_;
  std::vector<std::map<double, Position>> endEffectorPositions_;
  std::vector<std::map<double, Position>> endEffectorTargets_;
  std::vector<std::map<double, std::tuple<Position, Vector>>> surfaceNormals_;
  std::map<double, Stance> stances_;
  std::map<double, Pose> basePoses_;
  std::map<double, std::string> stepIds_;
};

} /* namespace free_gait */
