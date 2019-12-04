#pragma once

#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/PickupAction.h>
#include <ros/ros.h>
#include "planning_scene.h"

namespace tamp {
namespace planner {
using PickupPlanner = actionlib::SimpleActionClient<moveit_msgs::PickupAction>;

class MotionPlanner {
 public:
  MotionPlanner() = delete;

  static std::unique_ptr<MotionPlanner> MakeFromRosParam(
      const ros::NodeHandle &ph);

 private:
  MotionPlanner(const ros::NodeHandle &ph);

  std::shared_ptr<scene::PlanningScene> planning_scene_;

  std::unique_ptr<PickupPlanner> pickup_planner_;
};
}  // namespace planner
}  // namespace tamp