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

std::unique_ptr<MotionPlanner> MotionPlanner::MakeUniqueFromRosParam(
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
  if (!pickup_planner->waitForServer(ros::Duration(30))) {
    ROS_ERROR("Can not bring up pick up planner!");
    return nullptr;
  } else {
    ROS_INFO_STREAM("Pick up planner is up!");
  }
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

    tf2::Quaternion object_orientation;
    tf2::convert(object_to_pickup.primitive_poses[0].orientation,
                 object_orientation);
    tf2::Vector3 object_position;
    tf2::fromMsg(object_to_pickup.primitive_poses[0].position, object_position);
    tf2::Transform object_transform(object_orientation, object_position);
    tf2::Transform tf2_grasp_pose =
        object_transform * params_.grasps[i].grasp_pose;
    grasp_candidates[i].grasp_pose.pose.orientation =
        tf2::toMsg(tf2_grasp_pose.getRotation());
    tf2::toMsg(tf2_grasp_pose.getOrigin(),
               grasp_candidates[i].grasp_pose.pose.position);

    grasp_candidates[i].pre_grasp_approach.direction.header.frame_id =
        params_.world_frame;
    grasp_candidates[i].pre_grasp_approach.direction.vector =
        params_.grasps[i].approach_direction;
    grasp_candidates[i].pre_grasp_approach.min_distance =
        params_.grasps[i].approach_min_distance;
    grasp_candidates[i].pre_grasp_approach.desired_distance =
        params_.grasps[i].approach_desired_distance;

    const size_t num_gripper_joints =
        params_.grasps[i].gripper_joint_names.size();
    grasp_candidates[i].pre_grasp_posture.joint_names.resize(
        num_gripper_joints);
    grasp_candidates[i].pre_grasp_posture.points.resize(1);
    grasp_candidates[i].pre_grasp_posture.points[0].positions.resize(
        num_gripper_joints);
    for (size_t j = 0; j < num_gripper_joints; ++j) {
      grasp_candidates[i].pre_grasp_posture.joint_names[j] =
          params_.grasps[i].gripper_joint_names[j];
      grasp_candidates[i].pre_grasp_posture.points[0].positions[j] =
          params_.grasps[i].gripper_open_joint_positions[j];
    }
    grasp_candidates[i].pre_grasp_posture.points[0].time_from_start =
        ros::Duration(params_.grasps[i].gripper_time_from_start);

    grasp_candidates[i].post_grasp_retreat.direction.header.frame_id =
        params_.world_frame;
    grasp_candidates[i].post_grasp_retreat.direction.vector =
        params_.grasps[i].retreat_direction;
    grasp_candidates[i].post_grasp_retreat.min_distance =
        params_.grasps[i].retreat_min_distance;
    grasp_candidates[i].post_grasp_retreat.desired_distance =
        params_.grasps[i].retreat_desired_distance;

    grasp_candidates[i].grasp_posture.joint_names.resize(num_gripper_joints);
    grasp_candidates[i].grasp_posture.points.resize(1);
    grasp_candidates[i].grasp_posture.points[0].positions.resize(
        num_gripper_joints);
    for (size_t j = 0; j < num_gripper_joints; ++j) {
      grasp_candidates[i].grasp_posture.joint_names[j] =
          params_.grasps[i].gripper_joint_names[j];
      grasp_candidates[i].grasp_posture.points[0].positions[j] =
          params_.grasps[i].gripper_closed_joint_positions[j];
    }
    grasp_candidates[i].grasp_posture.points[0].time_from_start =
        ros::Duration(params_.grasps[i].gripper_time_from_start);
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

  std::vector<moveit_msgs::Grasp> grasp_candidates;
  if (!ConstructGrasp(pickup_object, grasp_candidates)) {
    ROS_ERROR_STREAM("Can not construct grasp poses for : " << pickup_object);
    return false;
  }
  goal.possible_grasps = grasp_candidates;
  goal.allow_gripper_support_collision = true;
  goal.planning_options.plan_only = params_.plan_only;
  goal.planning_options.planning_scene_diff = pickup_scene;
  goal.planning_options.replan = true;
  goal.planning_options.replan_attempts = params_.num_planning_attempts;
  goal.allowed_planning_time = params_.max_planning_time_sec;

  return true;
}

bool MotionPlanner::PlanPick(const std::vector<std::string>& scene_objects,
                             const std::string& pickup_object,
                             const std::string& pickup_object_from,
                             moveit_msgs::PickupResultConstPtr& plan_result) {
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
    ROS_INFO_STREAM("Planning finished with state " << state.toString());
    plan_result = nullptr;
    return true;
  }
  plan_result = pickup_planner_->getResult();
  return true;
}

}  // namespace planner
}  // namespace tamp