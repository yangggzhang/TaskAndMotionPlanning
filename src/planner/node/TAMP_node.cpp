#include "planner/task_motion_planner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "task_and_motion_planner");
  ros::NodeHandle nh, rh("~");
  std::unique_ptr<tamp::planner::TaskAndMotionPlanner> task_and_motion_planner =
      tamp::planner::TaskAndMotionPlanner::Make(rh);
  if (task_and_motion_planner == nullptr) {
    ROS_ERROR("Can not make task and motion planner");
    return -1;
  }
  ros::spin();
  return 0;
}