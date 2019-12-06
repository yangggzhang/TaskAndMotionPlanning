#include "planner/motion_executor_params.h"

namespace tamp {
namespace executor {

MotionExecutorParam::MotionExecutorParam() {}

bool MotionExecutorParam::LoadFromRosParams(ros::NodeHandle& ph) {
  if (!ph.getParam("move_group", move_group)) {
    ROS_ERROR_STREAM("Missing move group!");
    return false;
  }

  if (!ph.getParam("link_name", link_name)) {
    ROS_ERROR_STREAM("Missing link name!");
    return false;
  }

  if (!ph.getParam("touch_links", touch_links)) {
    ROS_ERROR_STREAM("Missing touch links!");
    return false;
  }

  return true;
}

}  // namespace executor
}  // namespace tamp