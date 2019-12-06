#pragma once

#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <string>
#include <vector>

namespace tamp {
namespace planner {

struct MoveItGrasp {
  tf2::Transform grasp_pose;
  geometry_msgs::Vector3 approach_direction;
  geometry_msgs::Vector3 retreat_direction;
  double approach_min_distance;
  double approach_desired_distance;
  double retreat_min_distance;
  double retreat_desired_distance;
  std::vector<std::string> gripper_joint_names;
  std::vector<double> gripper_open_joint_positions;
  std::vector<double> gripper_closed_joint_positions;
  double gripper_time_from_start;
};

class MotionPlannerParams {
 public:
  bool LoadFromRosParams(ros::NodeHandle& ph);

  std::string move_group;

  std::string world_frame;

  std::string planner_name;

  double max_planning_time_sec;

  bool plan_only;

  int num_planning_attempts;

  std::vector<MoveItGrasp> grasps;
};
}  // namespace planner
}  // namespace tamp