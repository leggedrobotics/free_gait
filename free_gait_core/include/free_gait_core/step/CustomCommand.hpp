/*
 * CustomCommand.hpp
 *
 *  Created on: Feb 15, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

// STD
#include <string>
#include <memory>

namespace free_gait {

class CustomCommand
{
 public:
  CustomCommand();
  virtual ~CustomCommand();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  std::unique_ptr<CustomCommand> clone() const;

  void setType(const std::string& type);
  void setDuration(const double duration);
  void setCommand(const std::string& command);

  const std::string& getType() const;
  double getDuration() const;
  const std::string& getCommand() const;

  friend std::ostream& operator << (std::ostream& out, const CustomCommand& customCommand);
  friend class StepCompleter;
  friend class StepRosConverter;

 private:
  std::string type_;
  double duration_;
  std::string command_;
};

} /* namespace */
