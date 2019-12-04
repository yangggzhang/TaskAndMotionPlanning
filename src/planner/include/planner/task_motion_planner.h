#pragma once

#include <ros/ros.h>
#include "planner/planning_scene.h"

namespace tamp {
namespace planner {
class TaskAndMotionPlanner {
 public:
  TaskAndMotionPlanner() = delete;

  static std::unique_ptr<TaskAndMotionPlanner> Make(const ros::NodeHandle& ph);

 private:
  TaskAndMotionPlanner(std::shared_ptr<scene::PlanningScene> planning_scene);

  std::shared_ptr<scene::PlanningScene> planning_scene_;
};
}  // namespace planner
}  // namespace tamp