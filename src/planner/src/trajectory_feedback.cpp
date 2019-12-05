#include "planner/trajectory_feedback.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <unordered_set>

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

bool TrajectoryFeedback::HasCollision(
    const std::string& pickup_object,
    const collision_detection::CollisionResult& collision_result,
    std::vector<std::string>& collided_objects_vec) {
  if (collision_result.collision) {
    std::unordered_set<std::string> collided_objects;
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for (it = collision_result.contacts.begin();
         it != collision_result.contacts.end(); ++it) {
      std::string object1 = it->first.first.c_str();
      std::string object2 = it->first.second.c_str();
      if (pickup_object.compare(object1) == 0 ||
          pickup_object.compare(object2)) {
        continue;
      }

      if (object1.size() < 5) {
        collided_objects.insert(object1);
      } else {
        std::string str_prefix = object1.substr(0, 5);
        if (str_prefix.compare(kPandaStr) != 0) {
          collided_objects.insert(object1);
        }
      }

      if (object2.size() < 5) {
        collided_objects.insert(object2);
      } else {
        std::string str_prefix = object2.substr(0, 5);
        if (str_prefix.compare(kPandaStr) != 0) {
          collided_objects.insert(object2);
        }
      }
    }
    if (!collided_objects.empty()) {
      for (auto obj = collided_objects.begin(); obj != collided_objects.end();
           ++obj) {
        collided_objects_vec.push_back(*obj);
      }
      return true;
    }
  }
  return false;
}

bool TrajectoryFeedback::GetTrajectoryFeedback(
    planning_scene::PlanningScenePtr& scene,
    robot_state::RobotState& current_state,
    const robot_model::JointModelGroup* arm,
    const std::vector<double>& arm_joints,
    const robot_model::JointModelGroup* hand,
    const std::vector<double>& hand_joints, const std::string& pickup_object,
    std::vector<std::string>& collided_objects) {
  if (!scene || !arm || !hand) {
    ROS_ERROR("Robot model is null for collision check!");
    return false;
  }
  collision_detection::AllowedCollisionMatrix acm =
      scene->getAllowedCollisionMatrix();
  current_state.setJointGroupPositions(arm, arm_joints);
  current_state.setJointGroupPositions(hand, hand_joints);
  robot_state::RobotState copied_state = scene->getCurrentState();
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.group_name = kHandGroup;
  collision_request.contacts = true;
  collision_request.max_contacts = kMaxContacts;
  scene->checkCollision(collision_request, collision_result, copied_state, acm);
  if (HasCollision(pickup_object, collision_result, collided_objects)) {
    return true;
  }
  collision_result.clear();
  collision_request.group_name = kArmGroup;
  scene->checkCollision(collision_request, collision_result, copied_state, acm);
  if (HasCollision(pickup_object, collision_result, collided_objects)) {
    return true;
  }
  return true;
}

bool TrajectoryFeedback::GetCollisionFeedback(
    const std::vector<std::string>& scene_objects,
    const std::string& pickup_object,
    moveit_msgs::PickupResultConstPtr& plan_result,
    std::vector<std::string>& collided_objects) {
  if (!plan_result) {
    ROS_ERROR("Empty motion planning result!");
    return false;
  }
  if (!planning_scene_interface_->ValidateScene(collided_objects)) {
    ROS_ERROR("The scene to get motion plan feedback is not valid!");
    return false;
  }
  if (plan_result->trajectory_stages.size() != kMotionPlanStage) {
    ROS_ERROR("The pick up plan does not have 5 stages!");
    return false;
  }

  std::vector<double> gripper_open_joints =
      plan_result->trajectory_stages[kOpenStage]
          .joint_trajectory.points.back()
          .positions;
  while (gripper_open_joints.size() < kHandJointDOF) {
    gripper_open_joints.push_back(gripper_open_joints.back());
  }

  std::vector<double> gripper_close_joints =
      plan_result->trajectory_stages[kCloseStage]
          .joint_trajectory.points.back()
          .positions;
  while (gripper_close_joints.size() < kHandJointDOF) {
    gripper_close_joints.push_back(gripper_close_joints.back());
  }

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene(
      new planning_scene::PlanningScene(robot_model));

  robot_state::RobotState& current_state =
      planning_scene->getCurrentStateNonConst();

  const robot_model::JointModelGroup* arm_model_group =
      current_state.getJointModelGroup(kArmGroup);
  const robot_model::JointModelGroup* hand_model_group =
      current_state.getJointModelGroup(kHandGroup);

  trajectory_msgs::JointTrajectory traj =
      plan_result->trajectory_stages[4].joint_trajectory;
  for (size_t i = traj.points.size() - 1; i >= 0; --i) {
    std::vector<double> arm_joint_state = traj.points[i].positions;
    if (!GetTrajectoryFeedback(planning_scene, current_state, arm_model_group,
                               arm_joint_state, hand_model_group,
                               gripper_close_joints, pickup_object,
                               collided_objects)) {
      ROS_ERROR("Can not check collision for trajectory!");
      return false;
    } else {
      if (!collided_objects.empty()) {
        ROS_INFO_STREAM("Found collisions between objects");
        for (const std::string& obj_in_collision : collided_objects)
          ROS_INFO_STREAM(obj_in_collision);
      }
      return true;
    }
  }

  traj = plan_result->trajectory_stages[2].joint_trajectory;
  for (size_t i = traj.points.size() - 1; i >= 0; --i) {
    std::vector<double> arm_joint_state = traj.points[i].positions;
    if (!GetTrajectoryFeedback(planning_scene, current_state, arm_model_group,
                               arm_joint_state, hand_model_group,
                               gripper_open_joints, pickup_object,
                               collided_objects)) {
      ROS_ERROR("Can not check collision for trajectory!");
      return false;
    } else {
      if (!collided_objects.empty()) {
        ROS_INFO_STREAM("Found collisions between objects");
        for (const std::string& obj_in_collision : collided_objects)
          ROS_INFO_STREAM(obj_in_collision);
      }
      return true;
    }
  }

  traj = plan_result->trajectory_stages[0].joint_trajectory;
  for (size_t i = traj.points.size() - 1; i >= 0; --i) {
    std::vector<double> arm_joint_state = traj.points[i].positions;
    if (!GetTrajectoryFeedback(planning_scene, current_state, arm_model_group,
                               arm_joint_state, hand_model_group,
                               gripper_open_joints, pickup_object,
                               collided_objects)) {
      ROS_ERROR("Can not check collision for trajectory!");
      return false;
    } else {
      if (!collided_objects.empty()) {
        ROS_INFO_STREAM("Found collisions between objects");
        for (const std::string& obj_in_collision : collided_objects)
          ROS_INFO_STREAM(obj_in_collision);
      }
      return true;
    }
  }

  return true;
}
}  // namespace feedback
}  // namespace tamp