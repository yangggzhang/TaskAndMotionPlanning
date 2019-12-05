#pragma once
#include<vector>
#include <string>
#include <moveit_msgs/PickupAction.h>

class MockPlanner {


bool PlanPick(const std::vector<std::string>& scene_objects,
                const std::string& pickup_object,
                const std::string& pickup_object_from,
                moveit_msgs::PickupResultConstPtr plan_result, int cnt);
  
bool GetCollisionFeedback(const std::vector<std::string>& scene_objects,
                            moveit_msgs::PickupResultConstPtr plan_result,
                            std::vector<std::string>& collided_objects,
                            int cnt);

};