#pragma once
#include <moveit_msgs/PickupAction.h>
#include <string>
#include <vector>

namespace tamp {
namespace planner {

class MockPlanner {
 public:
  static bool PlanPick(const std::vector<std::string>& scene_objects,
                       const std::string& pickup_object,
                       const std::string& pickup_object_from,
                       moveit_msgs::PickupResultConstPtr& plan_result, int cnt);

  static bool GetCollisionFeedback(
      const std::vector<std::string>& scene_objects,
      moveit_msgs::PickupResultConstPtr& plan_result,
      std::vector<std::string>& collided_objects, int cnt);
};

}  // namespace planner
}  // namespace tamp