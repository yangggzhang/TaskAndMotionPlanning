#pragma once
#include "task_planner_util.h"
#include <ros/ros.h>


namespace tamp {
namespace planner {


class TaskPlanner {
 public:
  TaskPlanner() = delete;
  static std::unique_ptr<TaskPlanner> MakeFromRosParam(const ros::NodeHandle &ph);

 private:
  TaskPlanner(const std::string& description_file);
};
}  // namespace planner
}  // namespace tamp