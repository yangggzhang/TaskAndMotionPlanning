#pragma once

#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <vector>

namespace tamp {
namespace scene {
class PlanningSceneParam {
 public:
  PlanningSceneParam();

  bool ParseFromRosParam(ros::NodeHandle &ph);

  std::vector<moveit_msgs::CollisionObject> GetCollisionObjects();

 private:
  std::vector<moveit_msgs::CollisionObject> collision_objects_;
};  // namespace scene
}  // namespace scene
}  // namespace tamp