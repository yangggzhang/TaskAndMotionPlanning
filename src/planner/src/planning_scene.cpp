#include "planner/planning_scene.h"
#include "planner/planning_utils.h"

namespace tamp {
namespace scene {
// static function
std::shared_ptr<PlanningScene> PlanningScene::MakeSharedFromRosParam(
    const ros::NodeHandle &ph) {
  PlanningSceneParam param;
  if (!param.ParseFromRosParam(ph)) {
    ROS_ERROR(
        "Failed to load collision objects information for planning scene!");
    return nullptr;
  } else
    return std::shared_ptr<PlanningScene>(new PlanningScene(param));
}

PlanningScene::PlanningScene(const PlanningSceneParam &param)
    : scene_param_(param) {
  scene_.reset(new moveit::planning_interface::PlanningSceneInterface());
  std::vector<moveit_msgs::CollisionObject> collision_objects =
      scene_param_.GetCollisionObjects();
  for (const auto &object : collision_objects) {
    collision_object_table_[object.id] = object;
  }
  scene_->applyCollisionObjects(collision_objects);
}

bool PlanningScene::GetObject(const std::string &object,
                              moveit_msgs::CollisionObject &object_info) {
  if (collision_object_table_.find(object) == collision_object_table_.end()) {
    ROS_ERROR_STREAM("Object : " << object
                                 << " does not exist in the planning scene !");
    return false;
  } else
    object_info = collision_object_table_[object];
}

bool PlanningScene::ValidateScene(
    const std::vector<std::string> &scene_objects) {
  for (const std::string &object : scene_objects) {
    if (collision_object_table_.find(object) == collision_object_table_.end()) {
      ROS_ERROR_STREAM(
          "Object : " << object << " does not exist in the planning scene !");
      return false;
    }
  }
  return true;
}

bool PlanningScene::GetPlanningScene(
    const std::vector<std::string> &scene_objects,
    moveit_msgs::PlanningScene &scene_msgs) {
  scene_msgs = moveit_msgs::PlanningScene();
  scene_msgs.robot_state.is_diff = false;
  scene_msgs.is_diff = true;
  for (const std::string &object : scene_objects) {
    if (collision_object_table_.find(object) == collision_object_table_.end()) {
      ROS_ERROR_STREAM(
          "Object : " << object << " does not exist in the planning scene !");
      return false;
    } else {
      scene_msgs.world.collision_objects.push_back(
          collision_object_table_[object]);
    }
  }
  return true;
}

bool PlanningScene::RemoveObject(const std::string &object) {
  if (collision_object_table_.find(object) == collision_object_table_.end()) {
    ROS_ERROR_STREAM("Object : " << object
                                 << " does not exist in the planning scene !");
    return false;
  } else {
    collision_object_table_.erase(object);
    return true;
  }
  return true;
}

bool PlanningScene::reset() {
  scene_.reset(new moveit::planning_interface::PlanningSceneInterface());
  std::vector<moveit_msgs::CollisionObject> collision_objects =
      scene_param_.GetCollisionObjects();
  if (collision_objects.empty()) {
    ROS_ERROR("Missing collision objects in the scene!");
    return false;
  }
  scene_->applyCollisionObjects(collision_objects);

  return true;
}

}  // namespace scene
}  // namespace tamp