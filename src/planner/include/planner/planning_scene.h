#pragma once

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>

namespace tamp {
namespace scene {
class PlanningScene {
 public:
  PlanningScene() = delete;

  std::unique_ptr<PlanningScene> MakeFromRosParam(const ros::NodeHandle &ph);

  bool reset();

 private:
  PlanningScene(
      const std::vector<moveit_msgs::CollisionObject> &collision_objects);

  bool LoadCollisionObjects(
      const ros::NodeHandle &ph,
      std::vector<moveit_msgs::CollisionObject> &collision_objects);

  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_;

  std::vector<moveit_msgs::CollisionObject> collision_objects_;
};
}  // namespace scene
}  // namespace tamp