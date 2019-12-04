#include "planner/trajectory_feedback.h"

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace tamp {
namespace feedback {

std::unique_ptr<TrajectoryFeedback> TrajectoryFeedback::MakeFromShared(
    std::shared_ptr<scene::PlanningScene> planning_scene_interface) {
  if (planning_scene_interface == nullptr) {
    return nullptr;
  } else
    return std::unique_ptr<TrajectoryFeedback>(
        new TrajectoryFeedback(std::move(planning_scene_interface)));
}

TrajectoryFeedback::TrajectoryFeedback(
    std::shared_ptr<scene::PlanningScene> planning_scene_interface)
    : planning_scene_interface_(std::move(planning_scene_interface)) {}

bool TrajectoryFeedback::GetCollisionFeedback(
    const std::vector<std::string>& scene_objects,
    moveit_msgs::PickupResultConstPtr plan_result,
    std::vector<std::string>& collided_objects) {
  if (!planning_scene_interface_->ValidateScene(collided_objects)) {
    ROS_ERROR("The scene to get motion plan feedback is not valid!");
    return false;
  }
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  for (size_t i = 0; i < plan_result->trajectory_stages.size(); ++i) {
    const trajectory_msgs::JointTrajectory& trajectory =
        plan_result->trajectory_stages[i].joint_trajectory;
    ROS_INFO_STREAM("Stage : " << i);
    for (const std::string& joint : trajectory.joint_names) {
      ROS_INFO_STREAM("Joint : " << joint);
    }
    ROS_INFO_STREAM("Point size : " << trajectory.points.size());
  }
  return true;
}
}  // namespace feedback
}  // namespace tamp