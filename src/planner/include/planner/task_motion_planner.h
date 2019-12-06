#pragma once

#include <ros/ros.h>
#include "planner/motion_planner.h"
#include "planner/planning_scene.h"
#include "planner/trajectory_feedback.h"
#include "task_planner_util.h"

namespace tamp {
namespace planner {
enum class PlannerStatus { SUCCESS, FAILED, REPLAN };

struct TmpOutput {
  PlannerStatus plan_status;  // Enum of planning status
  int fail_step_index;        // i means actions[i] failed
  std::vector<std::string>
      obstacles;  //<"C2", "C4"> obstacles that caused failure
};

class TaskAndMotionPlanner {
 public:
  TaskAndMotionPlanner() = delete;

  static std::unique_ptr<TaskAndMotionPlanner> Make(ros::NodeHandle& ph);

  TmpOutput interface(const std::vector<GroundedAction>& actions);

 private:
  TaskAndMotionPlanner(
      std::shared_ptr<scene::PlanningScene> planning_scene,
      std::unique_ptr<MotionPlanner> motion_planner,
      std::unique_ptr<feedback::TrajectoryFeedback> trajectory_feedback);

  std::shared_ptr<scene::PlanningScene> planning_scene_;
  std::unique_ptr<MotionPlanner> motion_planner_;
  std::unique_ptr<feedback::TrajectoryFeedback> trajectory_feedback_;
};

}  // namespace planner
}  // namespace tamp