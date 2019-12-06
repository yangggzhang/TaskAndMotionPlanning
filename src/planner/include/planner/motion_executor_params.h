#pragma once

#include <ros/ros.h>
#include <string>
#include <vector>

namespace tamp {
namespace executor {
class MotionExecutorParam {
 public:
  MotionExecutorParam();

  bool LoadFromRosParams(ros::NodeHandle &ph);

  std::string move_group;

  std::string link_name;

  std::vector<std::string> touch_links;

};  // namespace scene
}  // namespace executor
}  // namespace tamp