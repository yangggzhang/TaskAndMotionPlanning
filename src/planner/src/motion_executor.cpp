#include "planner/motion_executor.h"

namespace tamp {
namespace executor {

std::unique_ptr<MotionExecutor> MotionExecutor::MakeFromRosParam(
    ros::NodeHandle& ph,
    std::shared_ptr<scene::PlanningScene> planning_scene_interface) {
  std::string move_group;
  if (!ph.getParam("move_group", move_group)) {
    ROS_ERROR("Missing executor move group name!");
    return nullptr;
  }
  return std::unique_ptr<MotionExecutor>(
      new MotionExecutor(move_group, std::move(planning_scene_interface)));
}

bool MotionExecutor::ExecutePick(
    const std::string& pickup_object,
    const moveit_msgs::PickupResultConstPtr& plan_result) {
  return true;
}

MotionExecutor::MotionExecutor(
    const std::string& move_group_name,
    std::shared_ptr<scene::PlanningScene> planning_scene_interface)
    : planning_scene_interface_(std::move(planning_scene_interface)) {
  controller_.reset(new Controller(move_group_name));
}

}  // namespace executor
}  // namespace tamp