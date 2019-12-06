#pragma once

#include "planning_scene.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/PickupAction.h>
#include <ros/ros.h>
#include <string>

namespace tamp {
namespace executor {
using Controller = moveit::planning_interface::MoveGroupInterface;

class MotionExecutor {
 public:
  MotionExecutor() = delete;

  static std::unique_ptr<MotionExecutor> MakeFromRosParam(
      ros::NodeHandle& ph,
      std::shared_ptr<scene::PlanningScene> planning_scene_interface);

  bool ExecutePick(const std::string& pickup_object,
                   const moveit_msgs::PickupResultConstPtr& plan_result);

 private:
  MotionExecutor(
      const std::string& move_group_name,
      std::shared_ptr<scene::PlanningScene> planning_scene_interface);

  std::shared_ptr<scene::PlanningScene> planning_scene_interface_;

  std::unique_ptr<Controller> controller_;
};
}  // namespace executor
}  // namespace tamp