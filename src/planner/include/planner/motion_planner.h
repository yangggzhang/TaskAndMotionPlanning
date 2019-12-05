#pragma once

#include <actionlib/client/simple_action_client.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PickupAction.h>
#include <ros/ros.h>
#include <boost/optional.hpp>
#include <string>
#include <vector>

#include "motion_planner_params.h"
#include "planning_scene.h"

namespace tamp {
namespace planner {
using PickupPlanner = actionlib::SimpleActionClient<moveit_msgs::PickupAction>;

class MotionPlanner {
 public:
  MotionPlanner() = delete;

  static std::unique_ptr<MotionPlanner> MakeUniqueFromRosParam(
      const ros::NodeHandle& ph,
      std::shared_ptr<scene::PlanningScene> planning_scene_interface);

  // Return True 1. plan_result nullptr -> not available plan
  //             2. plan_result != nullptr available plan
  bool PlanPick(const std::vector<std::string>& scene_objects,
                const std::string& pickup_object,
                const std::string& pickup_object_from,
                moveit_msgs::PickupResultConstPtr plan_result);

 private:
  MotionPlanner(std::shared_ptr<scene::PlanningScene> planning_scene_interface,
                std::unique_ptr<PickupPlanner> planner,
                const MotionPlannerParams& params);

  bool ConstructPickupGoal(const std::vector<std::string>& scene_objects,
                           const std::string& pickup_object,
                           const std::string& pickup_object_from,
                           moveit_msgs::PickupGoal& goal);

  bool ConstructGrasp(const std::string& pickup_object,
                      std::vector<moveit_msgs::Grasp>& grasp_candidates);

  bool ResetScene(const std::vector<std::string>& scene_objects);

  std::shared_ptr<scene::PlanningScene> planning_scene_interface_;

  std::unique_ptr<PickupPlanner> pickup_planner_;

  MotionPlannerParams params_;
};
}  // namespace planner
}  // namespace tamp