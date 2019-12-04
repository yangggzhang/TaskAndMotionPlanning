#pragma once

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <unordered_map>

#include "planner/planning_scene_param.h"

namespace tamp {
namespace scene {
class PlanningScene {
 public:
  PlanningScene() = delete;

  static std::shared_ptr<PlanningScene> MakeSharedFromRosParam(
      const ros::NodeHandle &ph);

  bool ValidateScene(const std::vector<std::string> &scene_objects);

  bool GetPlanningScene(const std::vector<std::string> &scene_objects,
                        moveit_msgs::PlanningScene &scene_msgs);

  bool GetObject(const std::string &object,
                 moveit_msgs::CollisionObject &object_info);

  bool RemoveObject(const std::string &object);

  bool reset();

 private:
  PlanningScene(const PlanningSceneParam &param);

  bool LoadCollisionObjects(
      const ros::NodeHandle &ph,
      std::vector<moveit_msgs::CollisionObject> &collision_objects);

  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_;

  std::unordered_map<std::string, moveit_msgs::CollisionObject>
      collision_object_table_;

  PlanningSceneParam scene_param_;
};
}  // namespace scene
}  // namespace tamp