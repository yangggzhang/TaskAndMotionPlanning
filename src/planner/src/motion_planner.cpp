#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "planner/motion_planner.h"

namespace tamp {
namespace planner {

MotionPlanner::MotionPlanner(
    std::shared_ptr<scene::PlanningScene> planning_scene_interface,
    std::unique_ptr<PickupPlanner> planner, const MotionPlannerParams& params)
    : planning_scene_interface_(std::move(planning_scene_interface)),
      pickup_planner_(std::move(planner)),
      params_(params) {}

std::unique_ptr<MotionPlanner> MotionPlanner::MakeSharedFromRosParam(
    const ros::NodeHandle& ph,
    std::shared_ptr<scene::PlanningScene> planning_scene_interface) {
  MotionPlannerParams param;
  if (!param.LoadFromRosParams(ph)) {
    ROS_ERROR("Failed to load motion planner parameters!");
    return nullptr;
  }
  std::unique_ptr<PickupPlanner> pickup_planner =
      std::unique_ptr<PickupPlanner>(
          new PickupPlanner(param.planner_name, true));
  return std::unique_ptr<MotionPlanner>(new MotionPlanner(
      std::move(planning_scene_interface), std::move(pickup_planner), param));
}

bool MotionPlanner::ConstructGrasp(
    const std::string& pickup_object,
    std::vector<moveit_msgs::Grasp>& grasp_candidates) {
  moveit_msgs::CollisionObject object_to_pickup;
  if (!planning_scene_interface_->GetObject(pickup_object, object_to_pickup)) {
    ROS_ERROR("Can not retrieve info of the object to be picked up!");
    return false;
  }
  grasp_candidates.resize(params_.grasps.size());
  for (size_t i = 0; i < params_.grasps.size(); ++i) {
    grasp_candidates[i].grasp_pose.header.frame_id = params_.world_frame;
    tf2::Quaternion rotation;
    rotation.setRPY(params_.grasps[i].pose_orientation[0],
                    params_.grasps[i].pose_orientation[1],
                    params_.grasps[i].pose_orientation[2]);
    tf2::Quaternion object_tf_orientation;
    tf2::convert(object_to_pickup.primitive_poses[0].orientation,
                 object_tf_orientation);
  }
  return true;
}

bool MotionPlanner::ConstructPickupGoal(
    const std::vector<std::string>& scene_objects,
    const std::string& pickup_object, const std::string& pickup_object_from,
    moveit_msgs::PickupGoal& goal) {
  moveit_msgs::PlanningScene pickup_scene;
  if (!planning_scene_interface_->GetPlanningScene(scene_objects,
                                                   pickup_scene)) {
    ROS_ERROR("Can not set up planning scene for pick up action");
    return false;
  }

  goal.target_name = pickup_object;
  goal.group_name = params_.move_group;
  goal.support_surface_name = pickup_object_from;

  /// plan grasp
  goal.allow_gripper_support_collision = true;
  goal.planning_options.planning_scene_diff = pickup_scene;
  goal.planning_options.replan = true;
  goal.planning_options.replan_attempts = params_.num_planning_attempts;
  goal.allowed_planning_time = params_.max_planning_time_sec;

  return true;
}

bool MotionPlanner::PlanPick(const std::vector<std::string>& scene_objects,
                             const std::string& pickup_object,
                             const std::string& pickup_object_from) {
  if (!pickup_planner_->isServerConnected()) {
    ROS_ERROR("Pick up server is not connected!");
    return false;
  }

  moveit_msgs::PickupGoal pickup_goal;
  if (!ConstructPickupGoal(scene_objects, pickup_object, pickup_object_from,
                           pickup_goal)) {
    ROS_ERROR_STREAM("Can not construct pick up goal for : "
                     << pickup_object << " on " << pickup_object_from);
    return false;
  }

  const auto state = pickup_planner_->sendGoalAndWait(
      pickup_goal, ros::Duration(params_.max_planning_time_sec));
  if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("Pick up planning failed for : " << pickup_object << " on "
                                                     << pickup_object_from);
    return false;
  }
  moveit_msgs::PickupResultConstPtr result_ptr = pickup_planner_->getResult();
  return true;
}

}  // namespace planner
}  // namespace tamp