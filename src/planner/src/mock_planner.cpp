#include "planner/mock_planner.h"
#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace tamp {
namespace planner {
bool MockPlanner::PlanPick(const std::vector<std::string>& scene_objects,
                           const std::string& pickup_object,
                           const std::string& pickup_object_from,
                           moveit_msgs::PickupResultConstPtr& plan_result,
                           int cnt) {
  moveit_msgs::PickupResult dummy;
  switch (cnt) {
    case 0:
      // fail: p3 (o1,o2,o3)
      ROS_INFO("fail: p3 (o1,o2,o3)");
      plan_result = nullptr;
      break;
    case 1:
      // success: p3 (o3)
      ROS_INFO("success: p3 (o3)");
      plan_result = boost::make_shared<moveit_msgs::PickupResult>(dummy);
      break;
    case 2:
      // fail: p2 (o1,o2,o3)
      ROS_INFO("fail: p2 (o1,o2,o3)");
      plan_result = nullptr;
      break;
    case 3:
      // success: p2 (o2,o3)
      ROS_INFO("success: p2 (o2,o3)");
      plan_result = boost::make_shared<moveit_msgs::PickupResult>(dummy);
      break;
    case 4:
      // success: p1 (o1,o2,o3)
      ROS_INFO("success: p1 (o1,o2,o3)");
      plan_result = boost::make_shared<moveit_msgs::PickupResult>(dummy);
      break;
    case 5:
      // success: p1 (o1,o2,o3)
      ROS_INFO("success: p1 (o1,o2,o3)");
      plan_result = boost::make_shared<moveit_msgs::PickupResult>(dummy);
      break;
    case 6:
      // success: p1 (o1,o2,o3)
      ROS_INFO("success: p1 (o1,o2,o3)");
      plan_result = boost::make_shared<moveit_msgs::PickupResult>(dummy);
      break;
    case 7:
      // success: p1 (o1,o2,o3)
      ROS_INFO("success: p1 (o1,o2,o3)");
      plan_result = boost::make_shared<moveit_msgs::PickupResult>(dummy);
      break;
    default:
      plan_result = nullptr;
      std::cout << "error" << std::endl;
      break;
  }
  return true;
}

bool MockPlanner::GetCollisionFeedback(
    const std::vector<std::string>& scene_objects,
    moveit_msgs::PickupResultConstPtr& plan_result,
    std::vector<std::string>& collided_objects, int cnt) {
  switch (cnt) {
    case 0:
      ROS_INFO("collide C1, C4");
      collided_objects.push_back("C4");
      collided_objects.push_back("C1");
      break;
    case 1:
      ROS_INFO("collide C0");
      collided_objects.push_back("C0");
      break;
    default:
      std::cout << "error" << std::endl;
      break;
  }
  return true;
}

}  // namespace planner
}  // namespace tamp