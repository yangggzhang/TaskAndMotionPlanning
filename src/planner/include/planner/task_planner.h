#pragma once
#include <ros/ros.h>

#include "task_motion_planner.h"
#include "task_planner_util.h"

namespace tamp {
namespace planner {

class TaskPlanner {
 public:
  TaskPlanner() = delete;
  static std::unique_ptr<TaskPlanner> MakeFromRosParam(
      const ros::NodeHandle& ph);

  std::vector<GroundedAction> run(Heuristic heuristicOption = NoHeuristic);

 private:
  Env* env;
  TaskPlanner(const std::string& description_file,
              std::unique_ptr<TaskAndMotionPlanner> task_and_motion_planner);
  std::vector<GroundedAction> aStar(
      const GroundedConditionSet& startConditionSet,
      const GroundedConditionSet& goalConditionSet,
      const list<GroundedAction>& allPossibleActions,
      const Heuristic& heuristicOption, const bool& deletion = true,
      const bool& print = false);
  int getHeuristic(const Heuristic& heuristicOption,
                   const GroundedConditionSet& goalSet,
                   const GroundedConditionSet& currentSet,
                   const list<GroundedAction>& allPossibleActions);
  TmpOutput interface(std::vector<GroundedAction> actions);

  std::unique_ptr<TaskAndMotionPlanner> task_and_motion_planner_;
};
}  // namespace planner
}  // namespace tamp