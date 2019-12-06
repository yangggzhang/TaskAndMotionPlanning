#include "planner/planning_scene.h"
#include "planner/planning_utils.h"

namespace tamp {
namespace scene {
// static function
std::shared_ptr<PlanningScene> PlanningScene::MakeSharedFromRosParam(
    ros::NodeHandle &ph) {
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
  return true;
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
  // scene_msgs = moveit_msgs::PlanningScene();
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
  updateScene(scene_msgs.world.collision_objects);
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

std::vector<std::string> PlanningScene::getCollisionObjects() {
  std::vector<std::string> key_set;
  for (const auto &k : collision_object_table_) {
    key_set.push_back(k.first);
  }
  return std::move(key_set);
}
bool PlanningScene::updateScene(
    const std::vector<moveit_msgs::CollisionObject> &collision_objects) {
  resetScene();
  return scene_->applyCollisionObjects(collision_objects);
}

bool PlanningScene::resetScene() {
  auto objs = scene_->getKnownObjectNames();
  for (const auto &o : objs) {
    scene_->removeCollisionObjects(objs);
  }
  return true;
}

bool PlanningScene::ApplyScene(const std::vector<std::string> &scene_objects) {
  resetScene();
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  for (const std::string &object : scene_objects) {
    if (collision_object_table_.find(object) == collision_object_table_.end()) {
      ROS_ERROR_STREAM(
          "Object : " << object << " does not exist in the planning scene !");
      return false;
    } else {
      collision_objects.push_back(collision_object_table_[object]);
    }
  }
  return scene_->applyCollisionObjects(collision_objects);
}

}  // namespace scene
}  // namespace tamp