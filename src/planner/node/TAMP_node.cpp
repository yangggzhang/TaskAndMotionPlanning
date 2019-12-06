#include "planner/task_planner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "task_planner");
  ros::NodeHandle nh, rh("~");
  std::unique_ptr<tamp::planner::TaskPlanner> task_planner =
      tamp::planner::TaskPlanner::MakeFromRosParam(rh);
  if (task_planner == nullptr) {
    ROS_ERROR("Can not make task and motion planner");
    return -1;
  }
  ros::AsyncSpinner spinner(1);
  spinner.start();
  task_planner->run();
  return 0;
}