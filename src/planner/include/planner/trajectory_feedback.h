#pragma once

#include <moveit_msgs/PickupAction.h>
#include <string>
#include <vector>
#include "planning_scene.h"

namespace tamp {
namespace feedback {
class TrajectoryFeedback {
 public:
  TrajectoryFeedback() = delete;

  static std::unique_ptr<TrajectoryFeedback> MakeFromShared(
      std::shared_ptr<scene::PlanningScene> planning_scene_interface);

  bool GetCollisionFeedback(const std::vector<std::string>& scene_objects,
                            moveit_msgs::PickupResultConstPtr plan_result,
                            std::vector<std::string>& collided_objects);

 private:
  TrajectoryFeedback(
      std::shared_ptr<scene::PlanningScene> planning_scene_interface);

  std::shared_ptr<scene::PlanningScene> planning_scene_interface_;
};
}  // namespace feedback
}  // namespace tamp