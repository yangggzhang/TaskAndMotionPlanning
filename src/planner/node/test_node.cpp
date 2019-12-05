#include "planner/motion_planner.h"
#include "planner/planning_scene.h"
#include "planner/trajectory_feedback.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh, ph("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(1.0).sleep();

  std::shared_ptr<tamp::scene::PlanningScene> scene_ptr =
      tamp::scene::PlanningScene::MakeSharedFromRosParam(ph);

  if (scene_ptr == nullptr) {
    ROS_ERROR("Can not make scene interface!");
    return -1;
  }
  std::unique_ptr<tamp::planner::MotionPlanner> motion_planner_ptr =
      tamp::planner::MotionPlanner::MakeUniqueFromRosParam(ph, scene_ptr);
  if (motion_planner_ptr == nullptr) {
    ROS_ERROR("Can not make motion planner!");
    return -1;
  }
  std::unique_ptr<tamp::feedback::TrajectoryFeedback> trajectory_feedback_ptr =
      tamp::feedback::TrajectoryFeedback::MakeFromShared(scene_ptr);
  if (trajectory_feedback_ptr == nullptr) {
    ROS_ERROR("Can not make trajectory feedback!");
    return -1;
  }

  ros::WallDuration(1.0).sleep();

  std::string table = "table1";
  std::string object = "cylinder1";

  std::vector<std::string> scene_objects;
  scene_objects.push_back(table);
  scene_objects.push_back(object);
  moveit_msgs::PickupResultConstPtr plan_results;

  if (motion_planner_ptr->PlanPick(scene_objects, object, table,
                                   plan_results)) {
    ROS_INFO_STREAM("Get plan!");
  } else {
    ROS_INFO_STREAM("Can not get plan!");
  }

  ros::WallDuration(1.0).sleep();

  std::vector<std::string> collided_objects;

  if (trajectory_feedback_ptr->GetCollisionFeedback(
          scene_objects, object, plan_results, collided_objects)) {
    ROS_INFO_STREAM("Finish trajectory evaluation!");
  } else {
    ROS_INFO_STREAM("Finish trajectory evaluation failed!");
  }

  ros::waitForShutdown();
  return 0;
}