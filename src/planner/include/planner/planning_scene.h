#pragma once

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include "planner/planning_scene_param.h"

namespace tamp {
namespace scene {
class PlanningScene {
 public:
  PlanningScene() = delete;

  static std::unique_ptr<PlanningScene> MakeFromRosParam(
      const ros::NodeHandle &ph);

  bool reset();

 private:
  PlanningScene(const PlanningSceneParam &param);

  bool LoadCollisionObjects(
      const ros::NodeHandle &ph,
      std::vector<moveit_msgs::CollisionObject> &collision_objects);

  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_;

  PlanningSceneParam scene_param_;
};
}  // namespace scene
}  // namespace tamp