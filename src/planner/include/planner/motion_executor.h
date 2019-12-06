#pragma once

#include "motion_executor_params.h"
#include "planning_scene.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/PickupAction.h>
#include <ros/ros.h>
#include <string>

namespace tamp {
namespace executor {

const int KAttachStage = 4;
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
      const MotionExecutorParam& param,
      std::shared_ptr<scene::PlanningScene> planning_scene_interface);

  std::shared_ptr<scene::PlanningScene> planning_scene_interface_;

  std::unique_ptr<Controller> controller_;

  MotionExecutorParam param_;
};
}  // namespace executor
}  // namespace tamp