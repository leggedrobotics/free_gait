#pragma once

#include <free_gait_msgs/ExecuteStepsActionGoal.h>

namespace ros {
namespace message_traits {

namespace free_gait {
  static std::string frameId("base");
}

template<>
struct FrameId<free_gait_msgs::ExecuteStepsActionGoal, typename boost::enable_if<HasHeader<free_gait_msgs::ExecuteStepsActionGoal> >::type >
{
  static std::string* pointer(free_gait_msgs::ExecuteStepsActionGoal& m)
  {
    return &free_gait::frameId;
  }

  static std::string const* pointer(const free_gait_msgs::ExecuteStepsActionGoal& m)
  {
    return &free_gait::frameId;
  }

  static std::string value(const free_gait_msgs::ExecuteStepsActionGoal& m)
  {
    return free_gait::frameId;
  }
};

}
}
