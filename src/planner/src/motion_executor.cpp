#include "planner/motion_executor.h"

namespace tamp {
namespace executor {

std::unique_ptr<MotionExecutor> MotionExecutor::MakeFromRosParam(
    ros::NodeHandle& ph,
    std::shared_ptr<scene::PlanningScene> planning_scene_interface) {
  std::string move_group;
  MotionExecutorParam param;
  if (!param.LoadFromRosParams(ph)) {
    ROS_ERROR("Failed loading executor parameters!");
    return nullptr;
  }
  return std::unique_ptr<MotionExecutor>(
      new MotionExecutor(param, std::move(planning_scene_interface)));
}

bool MotionExecutor::ExecutePick(
    const std::string& pickup_object,
    const moveit_msgs::PickupResultConstPtr& plan_result) {
  ROS_INFO_STREAM("Trajectory total stage "
                  << plan_result->trajectory_stages.size());
  int num_stages = 4;
  if(pickup_object == "C3") {
    num_stages = 5;
  }
  for (int i = 0; i < num_stages; ++i) {
    ROS_INFO_STREAM("Execution : " << i);
    if (i == KAttachStage) {
      if (!planning_scene_interface_->AttachObjectToRobot(
              pickup_object, param_.link_name, param_.touch_links)) {
        ROS_ERROR_STREAM("Failed to attach : " << pickup_object
                                               << " to robot : "
                                               << param_.move_group);
        return false;
      }
    }
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    motion_plan.trajectory_ = plan_result->trajectory_stages[i];
    controller_->execute(motion_plan);
  }
  return true;
}

MotionExecutor::MotionExecutor(
    const MotionExecutorParam& param,
    std::shared_ptr<scene::PlanningScene> planning_scene_interface)
    : param_(param),
      planning_scene_interface_(std::move(planning_scene_interface)) {
  controller_.reset(new Controller(param.move_group));
}

}  // namespace executor
}  // namespace tamp