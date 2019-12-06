#include "planner/task_motion_planner.h"
#include "planner/mock_planner.h"

namespace tamp {
namespace planner {
static int test_cnt = 0;
static int test_cnt_cc = 0;
static bool delay_flag = 1;

TaskAndMotionPlanner::TaskAndMotionPlanner(
    std::shared_ptr<scene::PlanningScene> planning_scene,
    std::unique_ptr<MotionPlanner> motion_planner,
    std::unique_ptr<feedback::TrajectoryFeedback> trajectory_feedback,
    std::unique_ptr<executor::MotionExecutor> controller)
    : planning_scene_(std::move(planning_scene)),
      motion_planner_(std::move(motion_planner)),
      trajectory_feedback_(std::move(trajectory_feedback)),
      controller_(std::move(controller)) {}

std::unique_ptr<TaskAndMotionPlanner> TaskAndMotionPlanner::Make(
    ros::NodeHandle& ph) {
  std::shared_ptr<scene::PlanningScene> planning_scene =
      scene::PlanningScene::MakeSharedFromRosParam(ph);
  if (planning_scene == nullptr) {
    ROS_ERROR_STREAM("Cannot make planning scene!");
    return nullptr;
  }

  std::unique_ptr<MotionPlanner> motion_planner =
      MotionPlanner::MakeUniqueFromRosParam(ph, planning_scene);
  if (motion_planner == nullptr) {
    ROS_ERROR_STREAM("Cannot make motion planner!");
    return nullptr;
  }

  std::unique_ptr<feedback::TrajectoryFeedback> trajectory_feedback =
      feedback::TrajectoryFeedback::MakeFromShared(planning_scene);
  if (trajectory_feedback == nullptr) {
    ROS_ERROR_STREAM("Cannot make trajectory feedback!");
    return nullptr;
  }

  std::unique_ptr<tamp::executor::MotionExecutor> controller =
      executor::MotionExecutor::MakeFromRosParam(ph, planning_scene);
  if (controller == nullptr) {
    ROS_ERROR("Can not make controller!");
    return nullptr;
  }

  return std::unique_ptr<TaskAndMotionPlanner>(new TaskAndMotionPlanner(
      std::move(planning_scene), std::move(motion_planner),
      std::move(trajectory_feedback), std::move(controller)));
}

std::vector<std::string> generate_partial_scene(
    const std::vector<GroundedAction>& actions, int index) {
  // std::vector<std::string> res = {"Table"};
  std::vector<std::string> res;
  int size = actions.size();
  for (int i = index; i < size; ++i) {
    const auto& args = actions[i].get_arg_values();
    if (!args.empty()) {
      res.push_back(args.front());
    } else {
      ROS_ERROR_STREAM("Argument is empty in GroundedAction" +
                       actions[i].toString());
    }
  }
  res.push_back("Table");
  return res;
}

TmpOutput TaskAndMotionPlanner::interface(
    const std::vector<GroundedAction>& actions) {
  TmpOutput output;
  int size = actions.size();
  for (int i = 0; i < size; ++i) {
    auto& action = actions[i];
    std::vector<std::string> scene_objects =
        planning_scene_->getCollisionObjects();
    moveit_msgs::PickupResultConstPtr plan_result;
    const auto& args = action.get_arg_values();
    if (args.empty()) {
      ROS_ERROR_STREAM("Argument is empty in GroundedAction" +
                       action.toString());
    }
    ROS_INFO("Plan with original scene starts");
    motion_planner_->PlanPick(scene_objects, args.front(), "Table",
                              plan_result);
    ROS_INFO(" ");

    // MockPlanner::PlanPick(scene_objects, args.front(), "Table", plan_result,
    //                       test_cnt++);

    if (plan_result != nullptr) {
      // if (false) {
      // TODO: execute interface
      // execute(plan_result);
      // if (delay_flag) {
      //   delay_flag = 0;
      //   ros::Duration(2.8).sleep();
      // }
      // ros::Duration(3.8).sleep();
      ROS_INFO_STREAM("Start picking object : " << args.front());
      if (!controller_->ExecutePick(args.front(), plan_result)) {
        ROS_ERROR("Execution failed!");
        output.plan_status = PlannerStatus::FAILED;
        return output;
      }
      ROS_INFO_STREAM("Finished picking object : " << args.front());
      planning_scene_->RemoveObject(args.front());
      continue;
    } else {
      // plan with partial scene
      ROS_INFO("Plan with partial scene starts");
      std::vector<std::string> partial_scene_objects =
          generate_partial_scene(actions, i);
      motion_planner_->PlanPick(partial_scene_objects, args.front(), "Table",
                                plan_result);
      ROS_INFO(" ");
      // MockPlanner::PlanPick(partial_scene_objects, args.front(), "Table",
      //                       plan_result, test_cnt++);
      if (plan_result != nullptr) {
        // use collision checker to find which obj blocks the plan
        trajectory_feedback_->GetCollisionFeedback(
            scene_objects, args.front(), plan_result, output.obstacles);

        // MockPlanner::GetCollisionFeedback(scene_objects, plan_result,
        //                                   output.obstacles, test_cnt_cc++);
        output.plan_status = PlannerStatus::REPLAN;
        output.fail_step_index = i;
        return output;
      } else {
        output.plan_status = PlannerStatus::FAILED;
        return output;
      }
    }
  }
  output.plan_status = PlannerStatus::SUCCESS;
  return output;
}  // namespace planner

}  // namespace planner
}  // namespace tamp