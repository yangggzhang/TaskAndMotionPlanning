#include "planner/task_motion_planner.h"

namespace tamp {
namespace planner {
TaskAndMotionPlanner::TaskAndMotionPlanner(
    std::shared_ptr<scene::PlanningScene> planning_scene)
    : planning_scene_(std::move(planning_scene)) {}

std::unique_ptr<TaskAndMotionPlanner> TaskAndMotionPlanner::Make(
    const ros::NodeHandle& ph) {
  std::shared_ptr<scene::PlanningScene> planning_scene =
      scene::PlanningScene::MakeSharedFromRosParam(ph);
  if (planning_scene == nullptr) {
    return nullptr;
  }
  return std::unique_ptr<TaskAndMotionPlanner>(
      new TaskAndMotionPlanner(std::move(planning_scene)));
}

}  // namespace planner
}  // namespace tamp