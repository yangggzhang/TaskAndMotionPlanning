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
    return nullptr;
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