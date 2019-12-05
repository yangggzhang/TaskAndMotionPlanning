#include "planner/task_motion_planner.h"

namespace tamp {
namespace planner {
TaskAndMotionPlanner::TaskAndMotionPlanner(
    std::shared_ptr<scene::PlanningScene> planning_scene,
    std::unique_ptr<MotionPlanner> motion_planner,
    std::unique_ptr<feedback::TrajectoryFeedback> trajectory_feedback)
    : planning_scene_(std::move(planning_scene)), motion_planner_(std::move(motion_planner)), trajectory_feedback_(std::move(trajectory_feedback)) {}

std::unique_ptr<TaskAndMotionPlanner> TaskAndMotionPlanner::Make(
    const ros::NodeHandle& ph) {
  std::shared_ptr<scene::PlanningScene> planning_scene =
      scene::PlanningScene::MakeSharedFromRosParam(ph);
  if (planning_scene == nullptr) {
    return nullptr;
  }
  
  std::unique_ptr<MotionPlanner> motion_planner = MakeUniqueFromRosParam(ph, planning_scene_interface);
  if (motion_planner == nullptr) {
    return nullptr;
  }
  std::unique_ptr<feedback::TrajectoryFeedback> trajectory_feedback = MakeFromShared(planning_scene_interface);
  if (trajectory_feedback == nullptr) {
    return nullptr;
  }
  
  return std::unique_ptr<TaskAndMotionPlanner>(
      new TaskAndMotionPlanner(std::move(planning_scene), std::move(motion_planner), std::move(trajectory_feedback)));
}
  
std::vector<std::string> generate_partial_scene(const std::vector<GroundedAction>& actions, int index) {
  std::vector<std::string> res = {"Table"};
  const int size = actions.size();
  for(int i=index; i<size; ++i) {
    res.push_back(actions[i].get_arg_values()[0]);
  }
  return res;
}
  
TmpOutput TaskAndMotionPlanner::interface(const std::vector<GroundedAction>& actions) {
  TmpOutput output;
  const int size = actions.size();
  for (int i=0; i<size; ++i) {
    auto& action = actions[i];
    std::vector<std::string> scene_objects = planning_scene.getCollisionObjects();
    moveit_msgs::PickupResultConstPtr plan_result;
    motion_planner.PlanPick(scene_objects, action.get_arg_values()[0], "Table", plan_result);
    if(plan_result != nullptr) {
      //TODO: execute interface
      execute(plan_result);
      continue;
    }
    else  {
      //plan with partial scene
      std::vector<std::string> partial_scene_objects = generate_partial_scene(actions, i);
      motion_planner.PlanPick(partial_scene_objects, action.get_arg_values()[0], "Table", plan_result);
      if (plan_result != nullptr) {
        //use collision checker to find which obj blocks the plan
        //TODO: fix collision checker name
        output.obstacles = cc.collisionFeedback(scene_objects, plan_result);
        output.plan_status = PlannerStatus::REPLAN;
        output.fail_step_index = i;
        return output;
      }
      else {
        output.plan_status = PlannerStatus::FAILED;
        return output;
      }
    }
  }
  output.plan_status = PlannerStatus::SUCCESS;
  return output;
}
  
}  // namespace planner
}  // namespace tamp